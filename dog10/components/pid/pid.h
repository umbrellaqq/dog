#ifndef __PID_H__
#define __PID_H__

// 经典的 PID 结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float Target;       // 目标值（比如绝对水平就是 0 度）
    float Actual;       // 实际值（从 MPU6050 读回来的角度）
    
    float Error;        // 当前误差
    float LastError;    // 上次误差
    float Integral;     // 误差积分
    
    float Out;          // PID 计算输出结果
    float OutMax;       // 输出限幅最大值（防止计算爆炸把舵机干碎）
    float OutMin;       // 输出限幅最小值
    float IntegralMax;  // 积分限幅（防止长期倾斜导致积分饱和，电机疯转）
} PID_t;

// 初始化 PID 参数
void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max, float out_min, float integral_max);

// 计算并更新 PID（放入主循环）
float PID_Update(PID_t *pid, float target, float actual);

// 清空历史积分和误差（在跌倒恢复、重新站立时调用极其有用）
void PID_Reset(PID_t *pid);

#endif