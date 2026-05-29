#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "debug_tool.h"
#include "kinematics.h"
#include "gait_planner.h"
#include "bluetooth.h"
#include "mpu6050.h"
#include "pid.h"
#include "wifi_ap.h"
#include "web_server.h"
#include "nvs_flash.h"

extern void servo_smooth_task(void *arg);

extern bool is_force_90;
extern bool is_started;
extern float joy_val_x;
extern float joy_val_y;
extern bool is_cal_mode;

extern float g_body_pitch;
extern float g_body_height;
extern float g_body_offset_x;
extern float g_body_roll;
extern float g_body_yaw;

SemaphoreHandle_t g_state_mutex = NULL;

static const char *TAG = "MAIN_GAIT";

const float BASE_X[4] = {0, 0, 0, 0};
const float BASE_Y[4] = {-27.0f, -27.0f, -27.0f, -27.0f};
const float BASE_Z[4] = {110.0f, 110.0f, 110.0f, 110.0f};

// ============================================================
// 主控制任务（独立任务）
// ============================================================
void main_control_task(void *arg) {
    while (1) {
        LegCoords_t gait_delta;
        LegCoords_t attitude_delta;

        // 等待步态数据（阻塞）
        if (xQueueReceive(gait_queue, &gait_delta, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // 等待姿态数据（阻塞）
        if (xQueueReceive(attitude_queue, &attitude_delta, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        // 加锁读取模式标志
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool force_90 = is_force_90;
        bool started = is_started;
        xSemaphoreGive(g_state_mutex);

        if (force_90) {
            // 归中模式，主控制暂停，稍作延时避免忙等
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (!started) {
            // 待机锁定：步态增量清零
            for (int i = 0; i < 4; i++) {
                gait_delta.x[i] = 0.0f;
                gait_delta.y[i] = 0.0f;
                gait_delta.z[i] = 0.0f;
            }
        }

        // 融合基准坐标、步态、姿态和重心平移，填充足端目标结构体
        FootTargets_t foot;
        for (int i = 0; i < 4; i++) {
            foot.x[i] = BASE_X[i] + gait_delta.x[i] + attitude_delta.x[i] + g_body_offset_x;
            foot.y[i] = BASE_Y[i] + gait_delta.y[i] + attitude_delta.y[i];   // 已修正 Y 轴
            foot.z[i] = g_body_height + gait_delta.z[i] + attitude_delta.z[i];

            // 安全限幅
            if (foot.z[i] < 60.0f) foot.z[i] = 60.0f;
            else if (foot.z[i] > 160.0f) foot.z[i] = 160.0f;
        }

        // 将足端目标发送给 IK 任务（覆盖旧值）
        xQueueOverwrite(ik_queue, &foot);
    }
}

// ============================================================
// 主入口（仅负责初始化）
// ============================================================
void app_main(void)
{
    debug_system_init();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    load_config_from_nvs();

    ESP_LOGI(TAG, "========= 机器狗步态引擎启动 =========");

    wifi_init_softap();
    start_webserver();

    // ---- 创建舵机队列 ----
    servo_queue = xQueueCreate(1, sizeof(servo_target_t));
    if (servo_queue == NULL) {
        ESP_LOGE(TAG, "舵机队列创建失败");
        return;
    }

    // ---- 创建步态队列 ----
    gait_queue = xQueueCreate(GAIT_QUEUE_LENGTH, sizeof(LegCoords_t));
    if (gait_queue == NULL) {
        ESP_LOGE(TAG, "步态队列创建失败");
        return;
    }

    // ---- 创建姿态队列 ----
    attitude_queue = xQueueCreate(ATTITUDE_QUEUE_LENGTH, sizeof(LegCoords_t));
    if (attitude_queue == NULL) {
        ESP_LOGE(TAG, "姿态队列创建失败");
        return;
    }

    // ---- 创建 IK 队列 ----
    ik_queue = xQueueCreate(IK_QUEUE_LENGTH, sizeof(FootTargets_t));
    if (ik_queue == NULL) {
        ESP_LOGE(TAG, "IK 队列创建失败");
        return;
    }

    // ---- 创建互斥锁 ----
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "互斥锁创建失败");
        return;
    }

    ESP_LOGI(TAG, "底层初始化完毕，给你 3 秒钟时间把狗悬空拎起来...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // 初始站立姿态（手动设置，因为 IK 任务尚未启动）
    for (int i = 0; i < 4; i++) {
        set_leg_ik_position(i + 1, BASE_X[i], BASE_Y[i], BASE_Z[i]);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    servo_send_targets();   // 发送初始姿态

    ESP_LOGI(TAG, "四条腿已就位！3 秒后开始主循环...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    mpu_init();

    // 启动舵机平滑任务 (优先级 5)
    xTaskCreatePinnedToCore(servo_smooth_task, "smooth_task", 4096, NULL, 5, NULL, 1);

    // 启动步态任务 (优先级 3)
    xTaskCreatePinnedToCore(gait_task, "gait_task", 4096, NULL, 3, NULL, 1);

    // 启动姿态任务 (优先级 3)
    xTaskCreatePinnedToCore(attitude_task, "attitude_task", 4096, NULL, 3, NULL, 1);

    // 启动 IK 任务 (优先级 4，稍高以保证计算及时)
    xTaskCreatePinnedToCore(ik_task, "ik_task", 4096, NULL, 4, NULL, 1);

    // 启动主控制任务 (优先级 3，与生产者同级，或 4 视情况)
    xTaskCreatePinnedToCore(main_control_task, "main_control", 4096, NULL, 3, NULL, 1);

    ESP_LOGI(TAG, ">>> 核心逻辑运行中...");

    // 删除初始化任务本身，释放资源
    vTaskDelete(NULL);
}