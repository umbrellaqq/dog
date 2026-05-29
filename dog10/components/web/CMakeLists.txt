idf_component_register(
    SRCS "web_server.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server debug_tool pca9685 nvs_flash gait_planner # <-- nvs_flash 移到这里
    EMBED_FILES control.html cal.html                       # <-- 这里只放网页文件
)