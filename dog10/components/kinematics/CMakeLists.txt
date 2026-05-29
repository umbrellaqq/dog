# 注册 kinematics 组件
idf_component_register(
    
    SRCS "kinematics.c"
    INCLUDE_DIRS "."
    # 极其关键：声明依赖项！
    # 必须加上 debug_tool 和 pca9685，否则找不到它们的头文件
    REQUIRES debug_tool pca9685 gait_planner 
)

# 显式链接标准数学库 (m)，因为用到了 math.h 里的 sin, cos, acos 等函数
target_link_libraries(${COMPONENT_LIB} PUBLIC m)

