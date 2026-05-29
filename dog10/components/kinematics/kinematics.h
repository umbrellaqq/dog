#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "gait_planner.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// 舵机目标结构体
typedef struct {
    uint16_t pwm[12];
} servo_target_t;

extern QueueHandle_t servo_queue;

void servo_send_targets(void);
void servo_set_pwm(int index, float pwm);
float servo_get_pwm(int index);

// 姿态补偿队列和任务
#define ATTITUDE_QUEUE_LENGTH 1
extern QueueHandle_t attitude_queue;
void attitude_task(void *arg);

// ---------- 新增：IK 任务相关 ----------
// 四条腿的足端目标坐标
typedef struct {
    float x[4];
    float y[4];
    float z[4];
} FootTargets_t;

#define IK_QUEUE_LENGTH 1
extern QueueHandle_t ik_queue;
void ik_task(void *arg);
// ----------------------------------------

// 原有函数声明
void ik_single(float hu, float hl, float h, float x, float y, float z,
               float *gama, float *alpha, float *beta);
void set_leg_ik_position(int leg_id, float x, float y, float z);
LegCoords_t cal_attitude_ges(float pitch_deg, float roll_deg, float yaw_deg,
                             float x_shift, float y_shift);
void servo_smooth_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // __KINEMATICS_H__