#ifndef __WIFI_AP_H__
#define __WIFI_AP_H__

#include "esp_err.h"

// 在这里定义你专属的机器狗热点名字和密码
#define ESP_WIFI_SSID      "PA_Dog_WiFi"
#define ESP_WIFI_PASS      "12345678"
#define MAX_STA_CONN       4  // 最大允许连接的设备数（比如手机和电脑同时连）

void wifi_init_softap(void);

#endif