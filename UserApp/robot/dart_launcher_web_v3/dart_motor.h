/*
 * dart_motor.h — 四电机封装 (完全基于 DJImotor 库)
 *
 * 只做: 角色映射 / 模式(停/速度/角度/圈数) / 零点 / 方向 / 网页调参。
 * 实际的 PID 计算、CAN 收发、失联守护全部复用 dji_motor + controller + bsp_can。
 */
#ifndef DART_LAUNCHER_WEB_V3_DART_MOTOR_H
#define DART_LAUNCHER_WEB_V3_DART_MOTOR_H

#include <stdint.h>

#include "dji_motor.h"

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
  DJIMotorInstance* inst;
  float ratio;       // 转子:输出
  float zero;        // 零点 (转子 total_angle)
  uint8_t zero_valid;
  uint8_t online;
  uint32_t last_feed;
  DartMode_e mode;
  float target;      // 依 mode: rpm / deg / turns
} DartMotor_t;

extern DartMotor_t g_dart_motors[DART_MOTOR_COUNT];

void DartMotorInit(void);
void DartMotorTask(void);

void DartMotorSetCtrl(int slot, int mode, float value);
void DartMotorStopAll(void);
void DartMotorZero(int slot);
void DartMotorToggleDir(int slot);

/* 参数 id: 1速度Kp 2速度Ki 3速度Kd 4速度积分限幅 5速度输出限幅
 *          6角度Kp 7角度Ki 8角度Kd 9角度死区 10角度输出限幅 11减速比 (浮点*100) */
void DartMotorSetParam(int slot, int id, int value);
void DartMotorResetParams(int slot);
void DartMotorLoadParamRaw(int slot, int id, float value);
void DartMotorLoadZero(int slot, float zero, uint8_t valid);
void DartMotorLoadDir(int slot, uint8_t reverse);
void DartMotorReadParams(int slot, float* out);  // 11 项

float DartMotorOutAngle(int slot);  // 输出 deg
float DartMotorOutTurns(int slot);  // 输出 圈
int DartMotorAtTurns(int slot, float turns, float tol);

#endif  // DART_LAUNCHER_WEB_V3_DART_MOTOR_H
