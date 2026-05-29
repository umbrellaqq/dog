#include "pca9685.h"
#include "iic.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PCA";
// 🌟 新增：为 16 个通道各自建立一个“误差记账本”
// 用来记录每一帧因为硬件取整而“欠下”或“多给”的浮点数误差
static float pwm_error_acc[16] = {0.0f};
/**
 * @brief 初始化 PCA9685
 */
/*
esp_err_t pca_init(void) {
   // 写入 0x80 会触发软件重启所有寄存器
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, 0x80); 
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 然后再进入正常工作模式
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, 0x01);

    // 2. 设置舵机标准频率 50Hz
    pca_set_freq(50.0f);
    
    ESP_LOGI(TAG, "PCA9685 Init OK at 50Hz (Bus 1).");
    uint8_t mode1;
iic_read_bytes(I2C_BUS_PCA, PCA_ADDR, 0x00, &mode1, 1);
ESP_LOGI(TAG, "MODE1 = 0x%02x", mode1);  // 期望值应为 0x21 或 0x01 等

iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, 0x01);

    // 读取 MODE2 并打印
    uint8_t mode2;
    esp_err_t ret = iic_read_bytes(I2C_BUS_PCA, PCA_ADDR, 0x01, &mode2, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MODE2 = 0x%02x", mode2);  // 期望值 0x04 (推挽)
    } else {
        ESP_LOGE(TAG, "Failed to read MODE2");
    }

    pca_set_freq(50.0f);
    ESP_LOGI(TAG, "PCA9685 Init OK");
    return ESP_OK;
}*/


esp_err_t pca_init(void) {
    // 1. 软件复位 MODE1
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, 0x00); 
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. 强力设置 MODE2 为推挽输出 (0x04)
    // 这一步必须在设置频率之前，确保芯片在清醒状态下接收指令
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, 0x01, 0x04); 
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. 设置频率
    pca_set_freq(50.0f);

    // 4. 唤醒并开启自动增量 (注意：这里用 0x21，不要用 0xA1)
    // 0x21 代表: 唤醒状态(0), 响应全部调用(0), 保留(10), 自动增量(1), 正常工作(0)
   // iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, 0x21); 
   // vTaskDelay(pdMS_TO_TICKS(5));
// 4. 开启 AI 和 ALLCALL
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, 0x21);
      vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "PCA9685 初始化完成 (强力推挽模式已激活)");
    return ESP_OK;
}

/**
 * @brief 设置 PWM 频率 (舵机一般为 50Hz)
 */
esp_err_t pca_set_freq(float freq) {
    // 计算分频值
    float prescale_val = (25000000.0f / (4096.0f * freq)) - 1.0f;
    uint8_t prescale = (uint8_t)(prescale_val + 0.5f);

    uint8_t old_mode;
    iic_read_bytes(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, &old_mode, 1);/*保护现场，防止误操作其他配置*/
    
    // 必须进入 Sleep 模式才能修改频率
    uint8_t new_mode = (old_mode & 0x7F) | 0x10; 
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, new_mode);
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_PRESCALE, prescale);
    
    // 唤醒并等待振荡器稳定
    iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, old_mode);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 开启自动递增模式 (方便后续可能的连续写入)
    //iic_write_byte(I2C_BUS_PCA, PCA_ADDR, PCA_MODE1, old_mode | 0xa1); 
    /*多余的一步，开启自动递增应该由PCA9685int开启*/
    return ESP_OK;
}

// 连续写多个字节的函数
esp_err_t iic_write_bytes(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true); // 一气呵成写入数组
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief 设置指定通道的 PWM 脉宽
 */
esp_err_t pca_set_pwm(uint8_t ch, uint16_t on, uint16_t off) {
    uint8_t reg = PCA_LED0_ON_L + 4 * ch;
    
    // 把 4 个字节打包到一个数组里
    uint8_t data[4];
    data[0] = on & 0xFF;
    data[1] = on >> 8;
    data[2] = off & 0xFF;
    data[3] = off >> 8;
    
    // 只需要调用 1 次连续写，底层硬件会自动递增寄存器地址！绝对不会撕裂！
    return iic_write_bytes(I2C_BUS_PCA, PCA_ADDR, reg, data, 4);
}

/**
 * @brief 将角度 (0~90) 转化为符合规格书的 PWM 信号
 * @note 规格书：1000us-2000us 对应 0-90度
 */
/*
esp_err_t pca_set_angle(uint8_t ch, float angle) {
    // 安全校验，防止数组越界
    if (ch > 15) return ESP_ERR_INVALID_ARG; 

    // 按照规格书限幅
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 90.0f) angle = 90.0f;
    
    // 1. 算出包含小数的、绝对精确的虚拟 Ticks
    float exact_ticks = 205.0f + (angle / 90.0f) * (410.0f - 205.0f);
    
    // 2. 误差扩散：把上一帧被抹去的零头误差，加到这一帧里来补偿
    exact_ticks += pwm_error_acc[ch];
    
    // 3. 硬件强制输出：四舍五入变成整数
    uint16_t out_ticks = (uint16_t)(exact_ticks + 0.5f);
    
    // 4. 记账：算算这次输出后，我们又产生了多少微小的偏差，留给下一帧还
    pwm_error_acc[ch] = exact_ticks - (float)out_ticks;
    
    // 误差限幅防爆 (理论上它只会在 -0.5 到 +0.5 之间横跳，稍微给点余量)
    if (pwm_error_acc[ch] > 1.0f) pwm_error_acc[ch] = 1.0f;
    if (pwm_error_acc[ch] < -1.0f) pwm_error_acc[ch] = -1.0f;
    
    return pca_set_pwm(ch, 0, out_ticks);
}*/
esp_err_t pca_set_angle(uint8_t ch, float angle) {
    // 安全校验，防止数组越界
    if (ch > 15) return ESP_ERR_INVALID_ARG; 

    // 🌟 修改点 1：把物理限幅放宽到 180 度
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    
    // 🌟 修改点 2：替换为 180度 舵机的全新线性插值公式
    // 0度对应 102.4 ticks (500us)，180度对应 512.0 ticks (2500us)
    // 跨度是 512.0 - 102.4 = 409.6
    float exact_ticks = 102.4f + (angle / 180.0f) * 409.6f;
    
    // 2. 误差扩散：把上一帧被抹去的零头误差，加到这一帧里来补偿
    exact_ticks += pwm_error_acc[ch];
    
    // 3. 硬件强制输出：四舍五入变成整数
    uint16_t out_ticks = (uint16_t)(exact_ticks + 0.5f);
    
    // 4. 记账：算算这次输出后，我们又产生了多少微小的偏差，留给下一帧还
    pwm_error_acc[ch] = exact_ticks - (float)out_ticks;
    
    // 误差限幅防爆
    if (pwm_error_acc[ch] > 1.0f) pwm_error_acc[ch] = 1.0f;
    if (pwm_error_acc[ch] < -1.0f) pwm_error_acc[ch] = -1.0f;
    
    return pca_set_pwm(ch, 0, out_ticks);
}