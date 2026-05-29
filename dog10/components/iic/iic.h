#ifndef __IIC_H__
#define __IIC_H__

#include "driver/i2c.h"
#define I2C_BUS_MPU     I2C_NUM_0  
#define I2C_BUS_PCA     I2C_NUM_1  
  


#define MPU_SDA_PIN     21
#define MPU_SCL_PIN     22

#define PCA_SDA_PIN     18
#define PCA_SCL_PIN     19

// 必须添加此段，确保 C++ 编译器能识别 C 语言函数名
#ifdef __cplusplus
extern "C" {
#endif

// 你的总线定义和函数声明




esp_err_t iic_init_all(void);
esp_err_t iic_write_byte(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t data);
esp_err_t iic_read_bytes(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif