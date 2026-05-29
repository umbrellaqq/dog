idf_component_register(SRCS "mpu6050.c"
                       INCLUDE_DIRS "."
                       REQUIRES iic esp_timer) # 重点：声明它依赖 iic 模块