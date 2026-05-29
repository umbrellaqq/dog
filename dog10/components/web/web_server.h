#ifndef __WEB_SERVER_H__
#define __WEB_SERVER_H__

#include "esp_err.h"

// 启动 Web 服务器的接口
void start_webserver(void);
void save_config_to_nvs(void);
void load_config_from_nvs(void);

#endif