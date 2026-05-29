#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "esp_err.h"
#include <stdint.h>

// MPU6050 默认 I2C 地址
#define MPU6050_ADDR        0x68

// 核心寄存器定义
#define MPU6050_PWR_MGMT_1  0x6B  // 电源管理 1
#define MPU6050_WHO_AM_I    0x75  // 器件 ID
#define MPU6050_ACCEL_XOUT  0x3B  // 加速度计起始地址
#define MPU6050_GYRO_XOUT   0x43  // 陀螺仪起始地址

// 存储原始数据的结构体
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float temp;
} mpu_data_t;

// 函数声明
esp_err_t mpu_init(void);
esp_err_t mpu_read(mpu_data_t *data);
// ... 前面的代码保持不变 ...

// ✅ 新增：姿态解算函数 (直接输出用于平衡的 Pitch 和 Roll 角度)
esp_err_t mpu_get_angles(float *pitch, float *roll);

#endif