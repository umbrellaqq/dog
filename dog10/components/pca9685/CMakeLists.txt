idf_component_register(SRCS "pca9685.c"
                       INCLUDE_DIRS "."
                       REQUIRES iic) # 重点：声明它依赖 iic 模块