#ifndef __DEBUG_TOOL_H__
#define __DEBUG_TOOL_H__
#include <stdint.h>    // <--- 加上这一句，专门用来召唤 int16_t、uint8_t 等类型
// 机器狗配置结构体（用于保存校准数据）
typedef struct {
    int16_t offsets[12]; // 存储 12 个舵机的微调脉宽值
} dog_config_t;

// 声明全局变量，让外部文件也能认出它
extern dog_config_t g_dog_config;
#include "esp_err.h"
// 声明引脚翻译器函数
uint8_t map_J_to_PWM(uint8_t j_port);
// 动力总闸引脚定义
#define SERVO_PWR_PIN 25 

// 机器狗 12 自由度通道映射
// 格式: {肩部, 大腿, 小腿}
extern const uint8_t LEG_MAP[4][3];

/**
 * @brief 初始化调试系统（包含开启 GPIO 25 电源和 I2C 扫描）
 */
void debug_system_init(void);

/**
 * @brief 舵机逻辑归中（1500us，即规格书的中点位）
 */
void debug_servos_center(void);

/**
 * @brief 腿部识别测试（依次晃动 1-4 号腿，验证接线顺序）
 */
void debug_identify_legs(void);

/**
 * @brief 简单的原地深蹲动作测试（验证动力负载）
 */
void debug_squat_loop(void);
void save_config_to_nvs(void);
void load_config_from_nvs(void);
void load_config_from_nvs();
// 在头文件里声明这两个函数
void debug_servos_center_with_memory(void);
void debug_servos_center_pure_90(void);
#endif