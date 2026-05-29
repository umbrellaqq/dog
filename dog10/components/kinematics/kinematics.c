#include "kinematics.h"
#include "gait_planner.h"
#include <math.h>
#include "esp_log.h"
#include "debug_tool.h"
#include "pca9685.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "rom/ets_sys.h"
#include "freertos/semphr.h"
extern bool is_started;
extern SemaphoreHandle_t g_state_mutex;  // 用于访问 is_started

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define RAD_TO_DEG (180.0f / (float)PI)

// 机器狗真实物理尺寸 (单位: mm)
#define L1_THIGH 70.0f
#define L2_CALF  79.0f
#define L3_SHOU  27.0f
#define DOG_LENGTH 150.0f
#define DOG_WIDTH  100.0f

#define ANGLE_TO_PWM(deg) ((int)(102.0f + (deg) / 180.0f * 409.6f))
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

// ============================================================
// 模块内全局变量
// ============================================================

// 队列句柄
QueueHandle_t servo_queue = NULL;

// 目标 PWM 暂存区（主控制/标定函数写入，servo_send_targets 读取）
static float g_target_pwm[12] = {307,307,307, 307,307,307, 307,307,307, 307,307,307};

// ============================================================
// 辅助函数
// ============================================================

void servo_set_pwm(int index, float pwm) {
    if (index >= 0 && index < 12) {
        g_target_pwm[index] = pwm;
    }
}

float servo_get_pwm(int index) {
    if (index >= 0 && index < 12) {
        return g_target_pwm[index];
    }
    return 0.0f;
}

// 发送当前 g_target_pwm 到队列（覆盖旧值）
void servo_send_targets(void) {
    if (servo_queue == NULL) return;
    servo_target_t target;
    for (int i = 0; i < 12; i++) {
        target.pwm[i] = (uint16_t)(g_target_pwm[i] + 0.5f); // 四舍五入
    }
    xQueueOverwrite(servo_queue, &target);
}

// ============================================================
// 舵机平滑任务
// ============================================================
void servo_smooth_task(void *arg) {
    servo_target_t rx_target;   // 从队列收到的目标
    float g_current_pwm[12] = {307,307,307, 307,307,307, 307,307,307, 307,307,307};
    float g_target_local[12]  = {307,307,307, 307,307,307, 307,307,307, 307,307,307};

    while (1) {
        // 非阻塞接收最新目标（队列长度为1，所以不会阻塞）
        if (xQueueReceive(servo_queue, &rx_target, 0) == pdTRUE) {
            for (int i = 0; i < 12; i++) {
                g_target_local[i] = (float)rx_target.pwm[i];
            }
        }

        for (int i = 0; i < 12; i++) {
            int leg = i / 3;
            int joint = i % 3;
            uint8_t ch = LEG_MAP[leg][joint];

            // 安全读取 is_started
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            bool started = is_started;
            xSemaphoreGive(g_state_mutex);

            if (started) {
                if (g_current_pwm[i] != g_target_local[i]) {
                    g_current_pwm[i] = g_target_local[i];
                    int val = (int)g_current_pwm[i];
                    uint16_t on = (ch * 150) % 4096;
                    uint16_t off = (on + val) % 4096;
                    pca_set_pwm(ch, on, off);
                }
            } else {
                float diff = g_target_local[i] - g_current_pwm[i];
                if (fabs(diff) > 0.1f) {
                    float step = 0.0f;
                    float abs_diff = fabs(diff);
                    if (abs_diff > 100.0f) step = 8.0f;
                    else if (abs_diff > 30.0f) step = 4.0f;
                    else if (abs_diff > 7.0f) step = 2.0f;
                    else step = 0.5f;
                    if (step > abs_diff) step = abs_diff;
                    if (diff > 0) g_current_pwm[i] += step;
                    else          g_current_pwm[i] -= step;

                    int val = (int)g_current_pwm[i];
                    uint16_t on = (ch * 150) % 4096;
                    uint16_t off = (on + val) % 4096;
                    pca_set_pwm(ch, on, off);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================
// IK 相关函数（原样保留，未改动）
// ============================================================
float cal_test_shank(float x) {
    float p1 = 0.006649f;
    float p2 = 0.4414f;
    float p3 = 5.53f;
    return (p1 * x * x) + (p2 * x) + p3;
}

void ik_single(float hu, float hl, float h, float x, float y, float z,
               float *gama, float *alpha, float *beta) {
    float d = sqrtf(y * y + z * z);
    float d2_minus_h2 = (d * d) - (h * h);
    if (d2_minus_h2 < 0.0f) d2_minus_h2 = 0.0f;
    float l = sqrtf(d2_minus_h2);

    float gama2 = -atan2f(y, z);
    float gama1 = -atan2f(-h, l);
    *gama = RAD_TO_DEG * (gama2 - gama1);

    float s = sqrtf(l * l + x * x);
    float n = (s * s - hl * hl - hu * hu) / (2.0f * hu);

    float ratio_beta = n / hl;
    if (ratio_beta > 1.0f) ratio_beta = 1.0f;
    if (ratio_beta < -1.0f) ratio_beta = -1.0f;
    *beta = RAD_TO_DEG * (-acosf(ratio_beta));

    float alpha1 = -atan2f(x, l);
    float ratio_alpha = (hu + n) / s;
    if (ratio_alpha > 1.0f) ratio_alpha = 1.0f;
    if (ratio_alpha < -1.0f) ratio_alpha = -1.0f;
    float alpha2 = acosf(ratio_alpha);
    *alpha = RAD_TO_DEG * (alpha2 + alpha1);
}

void set_leg_ik_position(int leg_id, float x, float y, float z) {
    float shou_ang = 0.0f, ham_ang = 0.0f, shank_ang = 0.0f;
    ik_single(L1_THIGH, L2_CALF, L3_SHOU, x, y, z, &shou_ang, &ham_ang, &shank_ang);

    float comp_shank = cal_test_shank(-shank_ang);
    float final_shou = 90.0f, final_ham = 90.0f, final_shank = 90.0f;

    switch (leg_id) {
        case 1:
            final_shou  += shou_ang;
            final_ham   += ham_ang;
            final_shank = final_shank - 90.0f + comp_shank;
            break;
        case 2:
            final_shou  -= shou_ang;
            final_ham   -= ham_ang;
            final_shank = final_shank + 90.0f - comp_shank;
            break;
        case 3:
            final_shou  += shou_ang;
            final_ham   -= ham_ang;
            final_shank = final_shank + 90.0f - comp_shank;
            break;
        case 4:
            final_shou  -= shou_ang;
            final_ham   += ham_ang;
            final_shank = final_shank - 90.0f + comp_shank;
            break;
        default: return;
    }

    int pwm_shou  = ANGLE_TO_PWM(final_shou);
    int pwm_ham   = ANGLE_TO_PWM(final_ham);
    int pwm_shank = ANGLE_TO_PWM(final_shank);
    int idx = leg_id - 1;

    pwm_shou  += g_dog_config.offsets[idx * 3 + 0];
    pwm_ham   += g_dog_config.offsets[idx * 3 + 1];
    pwm_shank += g_dog_config.offsets[idx * 3 + 2];

    pwm_shou  = CLAMP(pwm_shou, 102, 512);
    pwm_ham   = CLAMP(pwm_ham, 102, 512);
    pwm_shank = CLAMP(pwm_shank, 102, 512);

    // 写入模块内目标数组（不直接发波）
    g_target_pwm[idx * 3 + 0] = pwm_shou;
    g_target_pwm[idx * 3 + 1] = pwm_ham;
    g_target_pwm[idx * 3 + 2] = pwm_shank;
}

LegCoords_t cal_attitude_ges(float pitch_deg, float roll_deg, float yaw_deg,
                             float x_shift, float y_shift) {
    LegCoords_t ges = {0};
    float P = pitch_deg * PI / 180.0f;
    float R = roll_deg * PI / 180.0f;
    float Y = yaw_deg * PI / 180.0f;
    float l = DOG_LENGTH;
    float b = DOG_WIDTH;

    ges.x[0] =  l/2 - x_shift - (l*cosf(P)*cosf(Y))/2 + (b*cosf(P)*sinf(Y))/2;
    ges.y[0] =  b/2 - y_shift - (b*(cosf(R)*cosf(Y)+sinf(P)*sinf(R)*sinf(Y)))/2 - (l*(cosf(R)*sinf(Y)-cosf(Y)*sinf(P)*sinf(R)))/2;
    ges.z[0] = -(b*(cosf(Y)*sinf(R)-cosf(R)*sinf(P)*sinf(Y)))/2 - (l*(sinf(R)*sinf(Y)+cosf(R)*cosf(Y)*sinf(P)))/2;

    ges.x[1] =  l/2 - x_shift - (l*cosf(P)*cosf(Y))/2 - (b*cosf(P)*sinf(Y))/2;
    ges.y[1] = -(b/2) - y_shift + (b*(cosf(R)*cosf(Y)+sinf(P)*sinf(R)*sinf(Y)))/2 - (l*(cosf(R)*sinf(Y)-cosf(Y)*sinf(P)*sinf(R)))/2;
    ges.z[1] =  (b*(cosf(Y)*sinf(R)-cosf(R)*sinf(P)*sinf(Y)))/2 - (l*(sinf(R)*sinf(Y)+cosf(R)*cosf(Y)*sinf(P)))/2;

    ges.x[2] = -(l/2) - x_shift + (l*cosf(P)*cosf(Y))/2 - (b*cosf(P)*sinf(Y))/2;
    ges.y[2] = -(b/2) - y_shift + (b*(cosf(R)*cosf(Y)+sinf(P)*sinf(R)*sinf(Y)))/2 + (l*(cosf(R)*sinf(Y)-cosf(Y)*sinf(P)*sinf(R)))/2;
    ges.z[2] =  (b*(cosf(Y)*sinf(R)-cosf(R)*sinf(P)*sinf(Y)))/2 + (l*(sinf(R)*sinf(Y)+cosf(R)*cosf(Y)*sinf(P)))/2;

    ges.x[3] = -(l/2) - x_shift + (l*cosf(P)*cosf(Y))/2 + (b*cosf(P)*sinf(Y))/2;
    ges.y[3] =  b/2 - y_shift - (b*(cosf(R)*cosf(Y)+sinf(P)*sinf(R)*sinf(Y)))/2 + (l*(cosf(R)*sinf(Y)-cosf(Y)*sinf(P)*sinf(R)))/2;
    ges.z[3] = -(b*(cosf(Y)*sinf(R)-cosf(R)*sinf(P)*sinf(Y)))/2 + (l*(sinf(R)*sinf(Y)+cosf(R)*cosf(Y)*sinf(P)))/2;

    return ges;
}
// ============================================================
// 姿态补偿任务（新增）
// ============================================================
QueueHandle_t attitude_queue = NULL;

void attitude_task(void *arg) {
    while (1) {
        // 加锁读取当前的姿态角和前后平移量
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        float pitch = g_body_pitch;
        float roll  = g_body_roll;
        float yaw   = g_body_yaw;
        float offset_x = g_body_offset_x;   // 平移量也可以纳入姿态输出，此处我们独立处理
        // 注意：height 并未使用，因为姿态补偿目前没有高度基准 Hc，后续可加
        xSemaphoreGive(g_state_mutex);

        // 计算姿态补偿（平移量在 main.c 融合，因此 x_shift 和 y_shift 仍传 0）
        LegCoords_t att = cal_attitude_ges(pitch, roll, yaw, 0.0f, 0.0f);

        // 将计算结果发送到队列（覆盖旧值）
        xQueueOverwrite(attitude_queue, &att);

        // 20ms 周期
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
// ============================================================
// IK 任务（新增）
// ============================================================
QueueHandle_t ik_queue = NULL;

void ik_task(void *arg) {
    FootTargets_t foot;
    while (1) {
        // 阻塞等待足端目标坐标
        if (xQueueReceive(ik_queue, &foot, portMAX_DELAY) == pdTRUE) {
            // 对每条腿执行逆运动学计算（结果写入内部 g_target_pwm）
            for (int i = 0; i < 4; i++) {
                set_leg_ik_position(i + 1, foot.x[i], foot.y[i], foot.z[i]);
            }
            // 将计算出的 12 路 PWM 发送到舵机队列
            servo_send_targets();
        }
    }
}