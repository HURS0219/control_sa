/*
 * dart_fsm.h — 自动化发射任务状态机
 *
 * 时序: 舵机标准 -> 拉簧标准 -> 拉簧预备 -> 舵机预备 -> 拉簧复位 -> 舵机复位
 */
#ifndef DART_LAUNCHER_WEB_V2_DART_FSM_H
#define DART_LAUNCHER_WEB_V2_DART_FSM_H

#include <stdint.h>

typedef enum {
  DART_YAW_MANUAL = 0,
  DART_YAW_VISION = 1,
  DART_YAW_GUIDE = 2,
} DartYawMode_e;

typedef enum {
  DART_TASK_IDLE = 0,
  DART_TASK_SERVO_STD1,
  DART_TASK_SPRING_STD1,
  DART_TASK_SPRING_PREP,
  DART_TASK_SERVO_PREP,
  DART_TASK_SPRING_BACK,
  DART_TASK_SERVO_STD2,
  DART_TASK_DONE,
} DartTaskStep_e;

extern float g_spring_turns;
extern int g_yaw_mode;
extern int g_estop;
extern int g_task_step;

void DartFsmInit(void);
void DartFsmTask(void);
void DartFsmServoGo(int which);
void DartFsmSpringGo(int which);
void DartFsmAutoStart(void);
void DartFsmAutoStop(void);
void DartFsmSetEstop(int on);

#endif  // DART_LAUNCHER_WEB_V2_DART_FSM_H
