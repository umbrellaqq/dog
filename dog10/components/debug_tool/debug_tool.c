#include "debug_tool.h"
#include "pca9685.h"
#include "esp_log.h"
#include <string.h>
#include "driver/gpio.h"
#include "iic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "kinematics.h" 
static const char *TAG = "DEBUG";

// ==========================================================
// 1. 网页防报错“空壳子” (必须保留，供 web_server.c 呼叫)
// ==========================================================
dog_config_t g_dog_config = {0};

// ==========================================================
// 2. 物理引脚映射表 (🌟 修复：去掉了非法的 extern)
// ==========================================================
const uint8_t LEG_MAP[4][3] = {
    { 4,  5,  6},   // 腿 1: 肩, 大腿, 小腿
    { 7,  8,  9},   // 腿 2: 肩, 大腿, 小腿
    {10, 11, 12},   // 腿 3: 肩, 大腿, 小腿
    {13, 14, 15}    // 腿 4: 肩, 大腿, 小腿
};

// ==========================================
// 💾 保存微调数据到 NVS (点击保存退出时调用)
// ==========================================
void save_config_to_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        nvs_set_blob(my_handle, "offsets", g_dog_config.offsets, sizeof(g_dog_config.offsets));
        nvs_commit(my_handle); 
        nvs_close(my_handle);
        ESP_LOGI("NVS", "✅ 舵机标定数据已永久保存！");
    } else {
        ESP_LOGE("NVS", "❌ 保存失败！");
    }
}

// ==========================================
// 📖 从 NVS 读取微调数据 (开机初始化时调用)
// ==========================================
void load_config_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(g_dog_config.offsets);
        err = nvs_get_blob(my_handle, "offsets", g_dog_config.offsets, &required_size);
        if (err == ESP_OK) {
            ESP_LOGI("NVS", "✅ 成功读取上次保存的标定数据！");
        } else {
            ESP_LOGW("NVS", "⚠️ 没找到标定数据，使用默认全 0 偏置");
        }
        nvs_close(my_handle);
    }
}


// ==========================================================
// 🌟 新增声明：引用在 kinematics.c 里定义的那个“目标脉宽数组”
// ==========================================================
extern float g_target_pwm[12]; 

// ==========================================================
// 功能 1：带记忆的归中 (保留历史微调量)
// ==========================================================
void debug_servos_center_with_memory(void) {
    ESP_LOGW(TAG, ">>> 执行带记忆的全局归中 (307 + NVS微调量) <<<");
    for (int l = 0; l < 4; l++) {
        int off_shou  = g_dog_config.offsets[l * 3 + 0];
        int off_ham   = g_dog_config.offsets[l * 3 + 1];
        int off_shank = g_dog_config.offsets[l * 3 + 2];
        servo_set_pwm(l * 3 + 0, 307 + off_shou);
        servo_set_pwm(l * 3 + 1, 307 + off_ham);
        servo_set_pwm(l * 3 + 2, 307 + off_shank);
    }
    servo_send_targets();   // 通过队列发送
}
// ==========================================================
// 功能 2：纯粹 90 度归中 (清空所有微调量，用于 t9 按键)
// ==========================================================
void debug_servos_center_pure_90(void) {
    ESP_LOGW(TAG, ">>> 执行全局 90 度归中：清空所有微调量 <<<");
    for (int i = 0; i < 12; i++) {
        g_dog_config.offsets[i] = 0;
        servo_set_pwm(i, 307);
    }
    servo_send_targets();   // 通过队列发送
}

// ==========================================================
// 下面的 test_all_servos_fast 和 debug_system_init 保持你原来的不变！
// ==========================================================

// ==========================================================
// 12 舵机极速并发测试函数 (修复寄生供电的黄金时序版)
// ==========================================================
void test_all_servos_fast(int pwm_shoulder, int pwm_thigh, int pwm_calf) {
    ESP_LOGI(TAG, "【极速下发】目标值 -> 肩:%d | 大腿:%d | 小腿:%d", pwm_shoulder, pwm_thigh, pwm_calf);
    
    // 用最快的速度把 12 个角度发出去 (I2C 极速写入)
    for (int l = 0; l < 4; l++) {
        pca_set_pwm(LEG_MAP[l][0], 0, pwm_shoulder); 
        vTaskDelay(pdMS_TO_TICKS(100)); // 肩
        pca_set_pwm(LEG_MAP[l][1], 0, pwm_thigh);
        vTaskDelay(pdMS_TO_TICKS(100)); // 大腿
        pca_set_pwm(LEG_MAP[l][2], 0, pwm_calf);  
        vTaskDelay(pdMS_TO_TICKS(100)); // 小腿
    }
}

// ================== 系统开机初始化 ==================
void debug_system_init(void) {
    ESP_LOGI(TAG, "Debug 系统初始化开始...");

    // 1. 【最关键的第一步】立刻接通真正的地线！
    gpio_reset_pin(25);
    gpio_set_direction(25, GPIO_MODE_OUTPUT);
    gpio_set_level(25, 1); 
    ESP_LOGI(TAG, "动力地线已接通！(寄生回路已被打破)");
// 🌟 修复 1：增加延时，确保 MOS 管完全导通，电容充满电
    vTaskDelay(pdMS_TO_TICKS(5000));
   

    // 2. 唤醒 I2C 和 PCA9685
    iic_init_all();
    pca_init(); 
    ESP_LOGI(TAG, "PCA9685 芯片唤醒成功！");

    
    
    ESP_LOGI(TAG, "初始化彻底完成！");
}
