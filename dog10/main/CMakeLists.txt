idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES iic mpu6050 pca9685 wifi web debug_tool nvs_flash kinematics gait_planner bluetooth pid) # 全部用小写！