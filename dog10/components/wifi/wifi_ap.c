#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_ap.h"
#include <string.h>
void wifi_init_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,           // <--- 使用头文件的宏
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = 1,
            .password = ESP_WIFI_PASS,       // <--- 使用头文件的宏
            .max_connection = MAX_STA_CONN,  // <--- 使用头文件的宏
            .authmode = WIFI_AUTH_WPA_WPA2_PSK // <--- 加上密码验证模式
        },
    };
    // 注意：如果密码长度为 0，才使用 WIFI_AUTH_OPEN 模式
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
    ESP_LOGI("WIFI", "WiFi AP 已开启，SSID: PYPY_DOG_C");
}