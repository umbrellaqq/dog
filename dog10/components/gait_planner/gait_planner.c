#include "gait_planner.h"
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"   // 因为要访问互斥锁保护的全局变量
#include "esp_log.h"
#include "freertos/semphr.h"
#define PI 3.14159265358979323846f

// 外部声明的互斥锁（在 main.c 中定义）
extern SemaphoreHandle_t g_state_mutex;

// 步态配置
GaitConfig_t g_gait_cfg = {
    .speed       = 0.05f,
    .step_height = 50.0f,
    .cg_shift_x  = 0.0f,
    .joy_y_ratio = 0.25f,
    .deadzone    = 5.0f
};

bool is_step_in_place_mode = false;
float g_body_height = 110.0f;
float g_body_offset_x = 0.0f;
float g_body_pitch = 0.0f;
float g_body_roll = 0.0f;
float g_body_yaw = 0.0f;

// 步态队列句柄
QueueHandle_t gait_queue = NULL;

// ------------------ 原有 Trot 步态算法 ------------------
LegCoords_t update_trot_gait(float phase, float step_length, float step_height) {
    LegCoords_t coords = {0};
    float p[4];
    p[0] = phase;
    p[1] = fmodf(phase + 0.5f, 1.0f);
    p[2] = phase;
    p[3] = fmodf(phase + 0.5f, 1.0f);

    for (int i = 0; i < 4; i++) {
        float t = p[i];
        if (t < 0.5f) {
            float ts = t * 2.0f;
            coords.x[i] = step_length * (ts - (1.0f/(2.0f*PI)) * sinf(2.0f*PI*ts)) - 0.5f*step_length;
            float b = 0.5f - 0.5f*cosf(2.0f*PI*ts);
            coords.z[i] = -step_height * b;
            coords.y[i] = 0.0f;
        } else {
            float ts = (t - 0.5f) * 2.0f;
            coords.x[i] = 0.5f*step_length - step_length * (ts - (1.0f/(2.0f*PI)) * sinf(2.0f*PI*ts));
            float b = 0.5f - 0.5f*cosf(2.0f*PI*ts);
            coords.z[i] = (step_height / 40.0f) * b;
            coords.y[i] = 0.0f;
        }
    }
    for (int i = 0; i < 4; i++) {
        coords.x[i] += g_gait_cfg.cg_shift_x;
    }
    return coords;
}

// Walk 步态（保留不变）
LegCoords_t update_walk_gait(float phase, float step_length, float step_height, float body_pitch) {
    LegCoords_t coords = {0};
    float sita = (-body_pitch * 0.5f) * PI / 180.0f;
    float sway_amp = 18.0f;
    float body_sway = sway_amp * sinf(2.0f * PI * phase);
    float offsets[4] = {0.0f, 0.5f, 0.25f, 0.75f};

    for (int i = 0; i < 4; i++) {
        float t = fmodf(phase + offsets[i], 1.0f);
        float x_raw = 0.0f, z_raw = 0.0f;
        if (t < 0.25f) {
            float ts = t * 4.0f;
            x_raw = step_length * (ts - (1.0f/(2.0f*PI)) * sinf(2.0f*PI*ts)) - 0.5f*step_length;
            z_raw = -step_height * (0.5f - 0.5f*cosf(2.0f*PI*ts));
        } else {
            float ts = (t - 0.25f) / 0.75f;
            x_raw = 0.5f*step_length - step_length * ts;
            z_raw = 0.0f;
        }
        float Hc = 110.0f;
        float x_rotated = (z_raw + Hc) * sinf(sita) + x_raw * cosf(sita);
        float z_rotated = (z_raw + Hc) * cosf(sita) - x_raw * sinf(sita) - Hc;
        coords.x[i] = x_rotated;
        coords.z[i] = z_rotated;
        coords.y[i] = body_sway;
    }
    return coords;
}

// ------------------ 步态任务（新增） ------------------
void gait_task(void *arg) {
    // 步态内部状态保持为任务内静态变量，隐藏起来
    static float internal_phase = 0.0f;

    while (1) {
        // 加锁读取摇杆和模式标志
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        float jx = 0.0f; // 暂时不用
        float jy = 0.0f;
        // 假设 joy_val_x, joy_val_y 定义在 web_server.c 或 main.c，这里进行外部声明
        extern float joy_val_x;
        extern float joy_val_y;
        jx = joy_val_x;
        jy = joy_val_y;
        bool step_in_place = is_step_in_place_mode;
        xSemaphoreGive(g_state_mutex);

        // 计算步长和抬腿高度（保留原逻辑）
        float step_len = 0.0f;
        float step_hgt = 0.0f;

        // 屏蔽 X 摇杆（原代码设置）
        jx = 0.0f;

        if (fabs(jy) < g_gait_cfg.deadzone && !step_in_place) {
            // 死区且非踏步，重置相位
            internal_phase = 0.0f;
            step_len = 0.0f;
            step_hgt = 0.0f;
        } else {
            if (step_in_place) {
                step_len = 0.0f;
            } else {
                step_len = jy * g_gait_cfg.joy_y_ratio;
            }
            step_hgt = g_gait_cfg.step_height;
            internal_phase += g_gait_cfg.speed;
            if (internal_phase >= 1.0f) {
                internal_phase -= 1.0f;
            }
        }

        // 目前只使用 Trot 步态
        LegCoords_t gait_delta = update_trot_gait(internal_phase, step_len, step_hgt);

        // 发送到队列（覆盖旧数据）
        xQueueOverwrite(gait_queue, &gait_delta);

        // 控制步态任务频率为 50Hz (20ms)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}