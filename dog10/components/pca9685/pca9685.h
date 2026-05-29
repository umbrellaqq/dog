#ifndef __PCA9685_H__
#define __PCA9685_H__

#include <stdint.h>
#include "esp_err.h"

// PCA9685 默认 I2C 地址
#define PCA_ADDR        0x40  

// 核心寄存器
#define PCA_MODE1       0x00  
#define PCA_PRESCALE    0xFE  
#define PCA_LED0_ON_L   0x06  

// 函数声明
esp_err_t pca_init(void);
esp_err_t pca_set_freq(float freq);
esp_err_t pca_set_pwm(uint8_t ch, uint16_t on, uint16_t off);
esp_err_t pca_set_angle(uint8_t ch, float angle);

#endif