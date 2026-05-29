idf_component_register(
    SRCS "gait_planner.c"
    INCLUDE_DIRS "."
)

# 必须链接 math 库
target_link_libraries(${COMPONENT_LIB} PUBLIC m)