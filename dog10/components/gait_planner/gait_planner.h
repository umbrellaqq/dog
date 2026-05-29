#ifndef __GAIT_PLANNER_H__
#define __GAIT_PLANNER_H__

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// 步态调参全局配置单
typedef struct {
    float speed;           // 步频
    float step_height;     // 抬腿高度
    float cg_shift_x;      // 重心前压补偿
    float joy_y_ratio;     // 摇杆 Y 轴比例
    float deadzone;        // 摇杆死区
} GaitConfig_t;

extern GaitConfig_t g_gait_cfg;

extern bool is_step_in_place_mode;
extern float g_body_height;
extern float g_body_offset_x;
extern float g_body_pitch;
extern float g_body_roll;
extern float g_body_yaw;

// 打包 4 条腿坐标的结构体
typedef struct {
    float x[4];
    float y[4];
    float z[4];
} LegCoords_t;

// 原有步态数学函数
LegCoords_t update_trot_gait(float phase, float step_length, float step_height);
LegCoords_t update_walk_gait(float phase, float step_length, float step_height, float body_pitch);

// ---------- 新增：步态任务和队列 ----------
#define GAIT_QUEUE_LENGTH 1      // 只保留最新数据
extern QueueHandle_t gait_queue; // 步态数据队列句柄
void gait_task(void *arg);       // 步态任务函数

#ifdef __cplusplus
}
#endif

#endif // __GAIT_PLANNER_H__