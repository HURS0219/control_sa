/*
 * dart_fsm.h — 发射架抽象状态机 + 自动化流程
 *
 * 舵机: 标准位 / 预备位
 * 拉簧: 标准位(0 圈) / 预备位(x 圈)
 * Yaw : 自瞄位(视觉闭环) / 制导位(0 度) / 手动
 *
 * 自动化时序 (对应文档第 12 条):
 *   1 yaw 人为选择  2 扳机电机人为设定  3 舵机标准位  4 拉簧标准位
 *   5 拉簧预备位    6 舵机预备位        7 拉簧标准位  8 舵机标准位
 */
#ifndef DART_LAUNCHER_WEB_DART_FSM_H
#define DART_LAUNCHER_WEB_DART_FSM_H

#include <stdint.h>

typedef enum {
  DART_YAW_MANUAL = 0,
  DART_YAW_VISION = 1,  // 自瞄位
  DART_YAW_GUIDE = 2,   // 制导位 (0 度)
} DartYawMode_e;

typedef enum {
  DART_AUTO_IDLE = 0,
  DART_AUTO_SERVO_STD1,
  DART_AUTO_SPRING_STD1,
  DART_AUTO_SPRING_PREP,
  DART_AUTO_SERVO_PREP,
  DART_AUTO_SPRING_BACK,
  DART_AUTO_SERVO_STD2,
  DART_AUTO_DONE,
} DartAutoStep_e;

extern float g_spring_turns;   // 预备位圈数 (输出轴)
extern int g_yaw_mode;         // DartYawMode_e
extern int g_estop;            // 1 = 急停
extern int g_auto_step;        // DartAutoStep_e

void DartFsmInit(void);
void DartFsmTask(void);

void DartFsmServoGo(int which);   // 0 标准位 / 1 预备位
void DartFsmSpringGo(int which);  // 0 标准位 / 1 预备位
void DartFsmAutoStart(void);
void DartFsmAutoStop(void);
void DartFsmSetEstop(int on);

#endif  // DART_LAUNCHER_WEB_DART_FSM_H
