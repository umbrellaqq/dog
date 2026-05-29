#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

// 暴露给 main.c 用的三个临时 PID 变量
extern float temp_kp;
extern float temp_ki;
extern float temp_kd;

// 初始化原生 BLE 的函数
void BlueSerial_Init(void);

// 往手机发送波形数据的函数 (BLE 版)
void BlueSerial_Printf(const char *format, ...);

#endif