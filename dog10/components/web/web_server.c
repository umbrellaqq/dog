#include <esp_http_server.h>
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "debug_tool.h"
#include "pca9685.h"
#include "gait_planner.h"
extern SemaphoreHandle_t g_state_mutex;
extern bool is_step_in_place_mode;

static const char *TAG = "WEB_CTRL";
bool is_force_90 = false; // 🌟 新增：强制 90 度十字架模式的开关
// 引用外部编译进来的 HTML 网页
extern const uint8_t control_html_start[] asm("_binary_control_html_start");
extern const uint8_t control_html_end[]   asm("_binary_control_html_end");
extern const uint8_t cal_html_start[]     asm("_binary_cal_html_start");
extern const uint8_t cal_html_end[]       asm("_binary_cal_html_end");

// 声明外部 NVS 存储函数
extern void save_config_to_nvs(void); 

// 全局状态记录
int current_leg = 0;      // 当前选中的腿 (0~3 对应 腿1~腿4)
bool is_cal_mode = false; // 是否在标定模式界面

// 🌟 开放给 main.c 使用的全局遥控变量
bool is_started = false;   // 启动开关状态
int current_gait = 0;      // 0: 小跑(转弯), 1: 平移(侧向)
float joy_val_x = 0.0f;    // 摇杆 X 轴 (-100 ~ 100)
float joy_val_y = 0.0f;    // 摇杆 Y 轴 (-100 ~ 100)

// ==========================================================
// 核心动作执行器：支持 90度微调 与 站立微调 双模式
// ==========================================================
static void adjust_servo(int leg_idx, int joint_idx, int step) {
    if (leg_idx < 0 || leg_idx > 3 || joint_idx < 0 || joint_idx > 2) return;
    
    // 1. 🌟 删掉之前那句 is_force_90 = false; 绝不自动解除 90 度模式！

    // 2. 更新微调量数据
    g_dog_config.offsets[leg_idx * 3 + joint_idx] += step;
    
    // 3. 🌟 核心逻辑：如果在 90 度模式，立刻发送【带微调的 90 度脉宽】！
    // 这样机器狗不仅不会站起来，还能在十字架状态下直观地看到微调效果！
    if (is_force_90) {
        debug_servos_center_with_memory(); 
    }

    ESP_LOGI(TAG, "微调: 腿 %d 关节 %d 偏置 -> %d", 
             leg_idx + 1, joint_idx, 
             g_dog_config.offsets[leg_idx * 3 + joint_idx]);
}
// ==========================================================
// 网页请求拦截器
// ==========================================================
esp_err_t main_get_handler(httpd_req_t *req) {
    char buf[128] = {0};
    char val[16] = {0};
    bool is_button_click = false; 

    // 解析 URL 参数
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {

        // 🌟 拦截高度调节
        if (httpd_query_key_value(buf, "hgt", val, sizeof(val)) == ESP_OK) {
            g_body_height = atof(val);
            ESP_LOGI(TAG, "高度调节: %.1f", g_body_height);
            httpd_resp_set_status(req, "204 No Content");
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
        // 🌟 拦截俯仰角调节 (Pitch)
        if (httpd_query_key_value(buf, "pit", val, sizeof(val)) == ESP_OK) {
            g_body_pitch = atof(val); // 将字符串转换为浮点数
            ESP_LOGI(TAG, "俯仰角调节: %.1f", g_body_pitch);
            httpd_resp_set_status(req, "204 No Content"); // 不刷新页面
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
// 🌟 拦截滚转角调节 (Roll)
        if (httpd_query_key_value(buf, "rol", val, sizeof(val)) == ESP_OK) {
            g_body_roll = atof(val); 
            ESP_LOGI(TAG, "滚转角调节: %.1f", g_body_roll);
            httpd_resp_set_status(req, "204 No Content"); 
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
        // 🌟 拦截偏航角调节 (Yaw)
        if (httpd_query_key_value(buf, "yaw", val, sizeof(val)) == ESP_OK) {
            g_body_yaw = atof(val); 
            ESP_LOGI(TAG, "偏航补偿调节: %.1f", g_body_yaw);
            httpd_resp_set_status(req, "204 No Content"); 
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
        // 🌟 拦截前后平移（重心）
        if (httpd_query_key_value(buf, "yst", val, sizeof(val)) == ESP_OK) {
            g_body_offset_x = atof(val);
            ESP_LOGI(TAG, "重心平移: %.1f", g_body_offset_x);
            httpd_resp_set_status(req, "204 No Content");
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
        
        // 🌟 拦截摇杆数据
        char val_f[16] = {0};
        char val_t[16] = {0};
        if (httpd_query_key_value(buf, "f", val_f, sizeof(val_f)) == ESP_OK &&
            httpd_query_key_value(buf, "t", val_t, sizeof(val_t)) == ESP_OK) {
            joy_val_y = atof(val_f); // 提取前后
            joy_val_x = atof(val_t); // 提取左右
            httpd_resp_set_status(req, "204 No Content");
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }

        // ==========================================================
        // 🛠️ 上半截：动作执行区
        // ==========================================================
         if (httpd_query_key_value(buf, "key", val, sizeof(val)) == ESP_OK) {
        is_button_click = true;
        int step_size = 2;
        if (strcmp(val, "start") == 0) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            is_started = true;
            xSemaphoreGive(g_state_mutex);
            ESP_LOGI(TAG, "🟢 机器狗解锁启动！");
        }
        else if (strcmp(val, "stop") == 0) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            is_started = false;
            xSemaphoreGive(g_state_mutex);
            ESP_LOGI(TAG, "🔴 机器狗安全锁定！");
        }
        else if (strcmp(val, "g0") == 0) { current_gait = 0; }
        else if (strcmp(val, "g1") == 0) { current_gait = 1; }
        else if (strcmp(val, "sn") == 0) { is_step_in_place_mode = true; }
        else if (strcmp(val, "sf") == 0) { is_step_in_place_mode = false; }
        else if (strcmp(val, "ss") == 0) {
            is_cal_mode = true;
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            is_force_90 = false;
            xSemaphoreGive(g_state_mutex);
            ESP_LOGI(TAG, "进入标定，开启边站边调！");
        }
        else if (strcmp(val, "sc") == 0) {
            save_config_to_nvs();
            is_cal_mode = false;
        }
        else if (strcmp(val, "t9") == 0) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            is_force_90 = true;
            xSemaphoreGive(g_state_mutex);
            debug_servos_center_pure_90();
            ESP_LOGI(TAG, "已触发绝对 90 度物理归中！");
        }
        else if (strcmp(val, "ts") == 0) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            is_force_90 = false;
            xSemaphoreGive(g_state_mutex);
            ESP_LOGI(TAG, "解除 90度十字架，恢复站立姿态测试！");
        }
            // 👇 ======== 腿部微调选择指令 ========
            else if (strcmp(val, "l1") == 0) current_leg = 0;
            else if (strcmp(val, "l2") == 0) current_leg = 1;
            else if (strcmp(val, "l3") == 0) current_leg = 2;
            else if (strcmp(val, "l4") == 0) current_leg = 3;
            else if (strcmp(val, "ju") == 0) adjust_servo(current_leg, 0, step_size);
            else if (strcmp(val, "jd") == 0) adjust_servo(current_leg, 0, -step_size);
            else if (strcmp(val, "hi") == 0) adjust_servo(current_leg, 1, step_size);
            else if (strcmp(val, "hd") == 0) adjust_servo(current_leg, 1, -step_size);
            else if (strcmp(val, "si") == 0) adjust_servo(current_leg, 2, step_size);
            else if (strcmp(val, "sd") == 0) adjust_servo(current_leg, 2, -step_size);
        }
    } // 👈 完美闭合的解析参数大括号

    // ==========================================================
    // 🌐 下半截：网页回复区
    // ==========================================================
    if (is_button_click) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_send(req, NULL, 0);
    } else {
        httpd_resp_set_type(req, "text/html");
        
        if (is_cal_mode) {
            // 发送标定页面的 HTML 上部
            httpd_resp_send_chunk(req, (const char *)cal_html_start, cal_html_end - cal_html_start);
            
            // 动态生成角度数据表
            char dynamic_table[1024];
            float angles[12];
            for(int i=0; i<12; i++) {
                angles[i] = 90.0f + ((float)g_dog_config.offsets[i] / 2.277f);
            }

          snprintf(dynamic_table, sizeof(dynamic_table),
                "<center><table border='1' style='width:90%%; text-align:center; border-collapse:collapse; background-color:#ffffff; font-size:18px; margin-top:20px;'>"
                "<tr style='background-color:#eee;'><th>部位</th><th>腿 1 (左前)</th><th>腿 2 (右前)</th></tr>"
                "<tr><td><b>肩/大/小</b></td><td>%.1f | %.1f | %.1f</td><td>%.1f | %.1f | %.1f</td></tr>"
                "<tr style='background-color:#eee;'><th>部位</th><th>腿 4 (左后)</th><th>腿 3 (右后)</th></tr>"
                "<tr><td><b>肩/大/小</b></td><td>%.1f | %.1f | %.1f</td><td>%.1f | %.1f | %.1f</td></tr>"
                "</table></center>"
                // 👇 🌟 把这里的按钮改成 ts (恢复站立)，背景改成绿色 (#7DFF7D)
                "<br><center><a href='/?key=ts'><button style='width:80%%; height:50px; font-size:18px; background:#7DFF7D; border-radius:10px;'>🐕 退出 90度，恢复站立测试</button></a></center><br><br>",
                angles[0], angles[1], angles[2],   
                angles[3], angles[4], angles[5],   
                angles[9], angles[10], angles[11], 
                angles[6], angles[7], angles[8]    
            );
            
            httpd_resp_send_chunk(req, dynamic_table, strlen(dynamic_table));
            
            // 发送结束标志
            httpd_resp_send_chunk(req, NULL, 0); 
        } else {
            // 发送控制页面的 HTML
            httpd_resp_send(req, (const char *)control_html_start, control_html_end - control_html_start);
        }
    }
    
    return ESP_OK; 
}

httpd_uri_t uri_get = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = main_get_handler,
    .user_ctx = NULL
};

void start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        ESP_LOGI(TAG, "重写版 Web 控制台启动成功！");
    }
}