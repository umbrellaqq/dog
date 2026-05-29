#include "mpu6050.h"
#include "iic.h"
#include "esp_log.h"
#include <math.h>
#include "esp_timer.h"

static const char *TAG = "MPU";

// 内部静态变量，用于保存上一时刻的姿态和时间
static float current_pitch = 0.0f;
static float current_roll = 0.0f;
static uint64_t last_time = 0;

/**
 * @brief 初始化 MPU6050
 */
esp_err_t mpu_init(void) {
    uint8_t check;
    // 读取 WHO_AM_I 寄存器 (0x75)
    iic_read_bytes(I2C_BUS_MPU, MPU6050_ADDR, MPU6050_WHO_AM_I, &check, 1);
    
    // 只要是 0x68 (标准6050) 或 0x70 (6500/9250) 都通过
    if (check != 0x68 && check != 0x70) {
        ESP_LOGE(TAG, "身份校验失败! 读到的是: 0x%02X", check);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "检测到传感器 ID: 0x%02X，初始化中...", check);

    // 唤醒芯片 (写 0x01 到 PWR_MGMT_1)
    iic_write_byte(I2C_BUS_MPU, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
    return ESP_OK;
}

/**
 * @brief 读取 MPU6050 的原始数据 (加速度+陀螺仪)
 */
esp_err_t mpu_read(mpu_data_t *data) {
    uint8_t buf[14];
    
    // 一次性读取 14 个寄存器 (Accel_X_H 到 Gyro_Z_L)
    esp_err_t err = iic_read_bytes(I2C_BUS_MPU, MPU6050_ADDR, MPU6050_ACCEL_XOUT, buf, 14);
    if (err != ESP_OK) return err;

    // 加速度计
    data->ax = (int16_t)((buf[0] << 8) | buf[1]);
    data->ay = (int16_t)((buf[2] << 8) | buf[3]);
    data->az = (int16_t)((buf[4] << 8) | buf[5]);
    
    // 温度
    int16_t temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->temp = (float)temp_raw / 340.0f + 36.53f;

    // 陀螺仪
    data->gx = (int16_t)((buf[8] << 8) | buf[9]);
    data->gy = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

/**
 * @brief 高级姿态解算：把生数据变成 Pitch 和 Roll 角度
 */
esp_err_t mpu_get_angles(float *out_pitch, float *out_roll) {
    mpu_data_t data;
    esp_err_t err = mpu_read(&data);
    if (err != ESP_OK) return err;

    // 1. 计算时间差 dt (积分需要)
    uint64_t now = esp_timer_get_time(); // 单位：微秒
    float dt = (last_time == 0) ? 0.02f : (now - last_time) / 1000000.0f;
    last_time = now;

    // 2. 利用加速度计计算绝对角度 (弧度转角度)
    float acc_roll = atan2f(data.ay, data.az) * 57.29578f;
    float acc_pitch = -atan2f(data.ax, sqrtf(data.ay * data.ay + data.az * data.az)) * 57.29578f;

    // 3. 利用陀螺仪计算角速度
    float gyro_rate_x = data.gx / 131.0f;
    float gyro_rate_y = data.gy / 131.0f;

    // 4. 互补滤波器融合
    current_roll = 0.98f * (current_roll + gyro_rate_x * dt) + 0.02f * acc_roll;
    current_pitch = 0.98f * (current_pitch + gyro_rate_y * dt) + 0.02f * acc_pitch;

    // 5. 输出结果
    *out_pitch = current_pitch;
    *out_roll = current_roll;

    return ESP_OK;
}