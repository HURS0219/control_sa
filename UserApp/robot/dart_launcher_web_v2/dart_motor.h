/*
 * dart_motor.h — 四电机驱动 (角色固定, 每电机独立参数/零点/方向)
 *
 * 模式: STOP / SPEED(输出rpm) / ANGLE(输出deg) / TURNS(输出圈数)
 * 位置量均以输出轴、相对“零点”计; 零点掉电保存, 也可网页取零点。
 */
#ifndef DART_LAUNCHER_WEB_V2_DART_MOTOR_H
#define DART_LAUNCHER_WEB_V2_DART_MOTOR_H

#include <stdint.h>

#include "simple_motor.h"

#define DART_MOTOR_COUNT 4

typedef enum {
  DART_ROLE_SPRING_A = 0,
  DART_ROLE_SPRING_B,
  DART_ROLE_TRIGGER,
  DART_ROLE_YAW,
} DartRole_e;

typedef enum {
  DART_MODE_STOP = 0,
  DART_MODE_SPEED = 1,  // 输出 rpm
  DART_MODE_ANGLE = 2,  // 输出 deg
  DART_MODE_TURNS = 3,  // 输出 圈
} DartMode_e;

typedef struct {
  float speed_ff;
  float speed_kp;
  float speed_ki;
  float speed_i_limit;
  float angle_kp;         // rpm / deg
  float angle_kd;         // 阻尼 (rpm / (deg/s))
  float angle_slew;       // 速度指令斜率 (rpm/s), <=0 关闭
  float gear_ratio;       // 转子:输出
  float angle_deadband;   // 输出 deg
  float angle_speed_max;  // 输出 rpm
  int32_t max_value;
} DartParam_s;

typedef struct {
  DartMode_e mode;
  float target;      // 依 mode: rpm / deg / turns
  float integral;
  float zero;        // dir*转子总角度 (零点)
  uint8_t zero_valid;
  uint8_t online;
  uint8_t reverse;
  uint8_t stalled;
  float speed_cmd;   // 角度环输出的速度指令(输出rpm)
  uint16_t run_ms;   // 本次运行时长(堵转保护用)
  uint16_t stall_ms; // 堵转累计
} DartCtrl_t;

extern DartParam_s g_dart_params[DART_MOTOR_COUNT];
extern DartCtrl_t g_dart_ctrl[DART_MOTOR_COUNT];
extern volatile uint8_t g_dart_scanning;

void DartMotorInit(void);
void DartMotorTask(void);

/* 控制 */
void DartMotorSetCtrl(int slot, int mode, float value);
void DartMotorStopAll(void);
void DartMotorZero(int slot);       // 当前位置=0
void DartMotorResetParams(int slot);

/* 参数/方向 */
void DartMotorSetParam(int slot, int id, int value);
void DartMotorToggleDir(int slot);
void DartMotorLoadParams(int slot, const DartParam_s* p);
void DartMotorLoadZero(int slot, float zero, uint8_t valid);

/* 查询 */
float DartMotorOutAngle(int slot);   // 输出 deg (相对零点)
float DartMotorOutTurns(int slot);   // 输出 圈
int DartMotorAtTurns(int slot, float turns, float tol);

#endif  // DART_LAUNCHER_WEB_V2_DART_MOTOR_H
