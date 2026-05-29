#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max, float out_min, float integral_max) 
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Target = 0.0f;
    pid->Actual = 0.0f;
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->Integral = 0.0f;
    pid->Out = 0.0f;
    pid->OutMax = out_max;
    pid->OutMin = out_min;
    pid->IntegralMax = integral_max;
}

float PID_Update(PID_t *pid, float target, float actual) 
{
    pid->Target = target;
    pid->Actual = actual;
    
    // 1. 计算比例项 (P) 误差
    pid->Error = pid->Target - pid->Actual;

    // 2. 计算积分项 (I) 误差，并进行严格限幅防爆
    pid->Integral += pid->Error;
    if (pid->Integral > pid->IntegralMax) {
        pid->Integral = pid->IntegralMax;
    } else if (pid->Integral < -pid->IntegralMax) {
        pid->Integral = -pid->IntegralMax;
    }

    // 3. 计算微分项 (D) 误差
    float derivative = pid->Error - pid->LastError;

    // 4. 计算最终输出
    pid->Out = (pid->Kp * pid->Error) + (pid->Ki * pid->Integral) + (pid->Kd * derivative);

    // 5. 对最终输出进行硬件级限幅防爆
    if (pid->Out > pid->OutMax) {
        pid->Out = pid->OutMax;
    } else if (pid->Out < pid->OutMin) {
        pid->Out = pid->OutMin;
    }

    // 6. 保存本次误差，供下次微分使用
    pid->LastError = pid->Error;

    return pid->Out;
}

void PID_Reset(PID_t *pid) 
{
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->Integral = 0.0f;
    pid->Out = 0.0f;
}