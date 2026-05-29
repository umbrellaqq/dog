#include "bluetooth.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

// 引入 NimBLE 核心库
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_SERIAL";

float temp_kp = 0.0f;
float temp_ki = 0.0f;
float temp_kd = 0.0f;

static uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_char_val_handle;

// 接收缓冲区 (防止 BLE 分包导致字符串断裂)
static char rx_buffer[256];
static int rx_index = 0;

static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_app_advertise(void);

// 发送波形数据给小程序的函数
void BlueSerial_Printf(const char *format, ...)
{
    if (ble_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buffer, len);
    ble_gatts_notify_custom(ble_conn_handle, tx_char_val_handle, om);
}

// ==========================================================
// 核心：处理小程序发来的 BLE 数据
// ==========================================================
static int ble_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // 遍历收到的每一个字节
        for (int i = 0; i < ctxt->om->om_len; i++) {
            char data = ctxt->om->om_data[i];
            
            // 遇到回车换行，开始解析
            if (data == '\n' || data == '\r') {
                if (rx_index > 0) {
                    rx_buffer[rx_index] = '\0'; 
                    
                    // ==========================================
                    // 完美移植的 STM32 strtok 逻辑
                    // ==========================================
                    char *Tag = strtok(rx_buffer, ",");
                    if (Tag != NULL && strcmp(Tag, "slider") == 0) {
                        char *Name = strtok(NULL, ",");
                        char *Value = strtok(NULL, ",");
                        if (Name != NULL && Value != NULL) {
                            if (strcmp(Name, "AngleKp") == 0) {
                                temp_kp = atof(Value);
                                ESP_LOGI(TAG, "=> 更新 Kp: %.2f", temp_kp);
                            }
                            else if (strcmp(Name, "AngleKi") == 0) {
                                temp_ki = atof(Value);
                                ESP_LOGI(TAG, "=> 更新 Ki: %.2f", temp_ki);
                            }
                            else if (strcmp(Name, "AngleKd") == 0) {
                                temp_kd = atof(Value);
                                ESP_LOGI(TAG, "=> 更新 Kd: %.2f", temp_kd);
                            }
                        }
                    }
                    rx_index = 0; // 解析完清空缓冲区
                }
            } else {
                if (rx_index < sizeof(rx_buffer) - 1) {
                    rx_buffer[rx_index++] = data;
                }
            }
        }
    }
    return 0;
}

// ==========================================================
// GATT 表：模拟 HC-08/JDY-23 等常见 BLE 透传模块 (FFE0/FFE1)
// ==========================================================
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFFE0), // 透传服务
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0xFFE1), // 透传特征值
                .access_cb = ble_svc_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &tx_char_val_handle,
            },
            { 0 }
        }
    },
    { 0 }
};

// 蓝牙连接与广播事件
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "小程序已连接!");
                ble_conn_handle = event->connect.conn_handle;
            } else {
                ble_app_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "小程序已断开，重新广播...");
            ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise();
            break;
    }
    return 0;
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name = "RobotDog_BLE"; // 这是小程序搜到的名字！

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

static void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, NULL);
    ble_app_advertise();
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void BlueSerial_Init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "原生 BLE 透传已启动！");
}