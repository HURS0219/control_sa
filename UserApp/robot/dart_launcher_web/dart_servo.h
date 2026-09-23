/*
 * dart_servo.h — 扳机舵机 (PWM1 / TIM1_CH1) 标准位 / 预备位
 */
#ifndef DART_LAUNCHER_WEB_DART_SERVO_H
#define DART_LAUNCHER_WEB_DART_SERVO_H

#include <stdint.h>

#define DART_SERVO_STD  0
#define DART_SERVO_PREP 1

extern float g_servo_std_us;   // 标准位脉宽
extern float g_servo_prep_us;  // 预备位脉宽
extern float g_servo_cur_us;   // 当前脉宽
extern int g_servo_state;      // 0 标准位 / 1 预备位

void DartServoInit(void);
void DartServoSetPos(int which, float us);  // 设定标准/预备位脉宽
void DartServoGo(int which);                // 转到标准/预备位
void DartServoSetUs(float us);              // 直接给脉宽
void DartServoTask(void);

#endif  // DART_LAUNCHER_WEB_DART_SERVO_H
