/*
 * dart_motor.h — 四电机角色抽象 + 每电机独立 PID 参数
 *
 * 四个固定角色 (下标 = 角色): 0 拉簧A, 1 拉簧B, 2 扳机丝杆, 3 Yaw
 * 每个角色一套独立参数, 可在网页上在线整定并掉电保存。
 * 速度/角度均以【输出轴】为单位 (rpm / deg)。
 */
#ifndef DART_LAUNCHER_WEB_DART_MOTOR_H
#define DART_LAUNCHER_WEB_DART_MOTOR_H

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
  DART_CTRL_STOP = 0,
  DART_CTRL_SPEED = 1,
  DART_CTRL_ANGLE = 2,
} DartCtrlMode_e;

/* 单个电机的一套 PID / 机械参数 */
typedef struct {
  float speed_ff;        // 速度前馈
  float speed_kp;
  float speed_ki;
  float speed_i_limit;   // 积分限幅
  float angle_kp;        // 角度环 Kp (输出 rpm / deg)
  float angle_kd;        // 角度环 Kd
  float angle_slew;      // 角度环输出限速 (rpm)
  float gear_ratio;      // 转子:输出
  float angle_deadband;  // 角度死区 (输出 deg)
  float angle_speed_max; // 角度环输出上限 (输出 rpm)
  int32_t max_value;     // 最终电流/电压硬限幅
} DartParam_s;

/* 单个电机的运行状态 */
typedef struct {
  DartCtrlMode_e mode;
  float target;     // SPEED: 输出 rpm ; ANGLE: 输出 deg
  float integral;   // 速度环积分
  float zero;       // 角度零点 (电机总角度, 转子 deg)
  uint8_t zero_valid;  // 零点是否已确定 (掉电保存或首次反馈)
  uint8_t online;   // 是否有反馈
  uint8_t reverse;  // 方向
} DartCtrl_t;

extern DartParam_s g_dart_params[DART_MOTOR_COUNT];
extern DartCtrl_t g_dart_ctrl[DART_MOTOR_COUNT];
extern int g_dart_sel;                     // 当前调参槽位
extern volatile uint8_t g_dart_scanning;   // 1 = 扫描中, 暂停普通 CAN 读取

void DartMotorInit(void);
void DartMotorTask(void);

int DartMotorSlotOfRole(DartRole_e role);

/* 控制接口 */
void DartMotorSetCtrl(int slot, int mode, float value);
void DartMotorStopAll(void);
void DartMotorZero(int slot);  // slot < 0 -> 全部

/* 参数 (作用于 g_dart_sel) */
void DartMotorSelect(int slot);
void DartMotorSetParam(int id, int value);
void DartMotorResetParams(int slot);  // 恢复默认参数

void DartMotorStoreActive(void);
void DartMotorLoadActive(void);

/* 供状态机使用: 判断电机是否到达目标 (输出轴圈数) */
int DartMotorAtTargetTurns(int slot, float turns, float tol);

#endif  // DART_LAUNCHER_WEB_DART_MOTOR_H
