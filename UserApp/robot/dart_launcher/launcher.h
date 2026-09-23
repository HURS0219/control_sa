/*
 * launcher.h — 飞镖发射架顶层数据结构与接口
 */
#ifndef DART_LAUNCHER_LAUNCHER_H
#define DART_LAUNCHER_LAUNCHER_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_pwm.h"
#include "dji_motor.h"
#include "remote_control.h"

/* 顶层模式 (由遥控器右侧拨杆切换) */
typedef enum {
  LAUNCHER_MODE_DEBUG = 0,  // 下档: 纯手动调试各电机/舵机
  LAUNCHER_MODE_CALIB,      // 中/上档且未校准: 自动找零
  LAUNCHER_MODE_READY,      // 已校准, 待命
  LAUNCHER_MODE_AUTO,       // 上档: 蓄力->发射时序
  LAUNCHER_MODE_ESTOP       // 急停
} LauncherMode_e;

/* 校准子步骤 */
typedef enum {
  CALI_IDLE = 0,
  CALI_CHARGE_RETRACT,  // 蓄力机构找限位
  CALI_CHARGE_BACK,     // 回退消形变
  CALI_AIM_RETRACT,     // yaw 找限位
  CALI_AIM_BACK,        // 回退到中位
  CALI_DONE
} LauncherCaliStep_e;

/* 自动发射子步骤 */
typedef enum {
  AUTO_IDLE = 0,
  AUTO_CHARGING,  // 蓄力到目标能量
  AUTO_CHARGED,   // 到位稳定
  AUTO_FIRING,    // 舵机释放扳机
  AUTO_DONE       // 完成/保持
} LauncherAutoStep_e;

typedef struct {
  /* ---- 蓄力: 2 x M3508 ---- */
  DJIMotorInstance* charge_master;
  DJIMotorInstance* charge_slave;
  float charge_zero_master;    // 主电机找零零点(转子角度)
  float charge_zero_slave;     // 从电机找零零点(转子角度)
  float charge_target_delta;   // 目标相对零点的角度增量(机械同向, 转子deg)
  float charge_draw_mm;        // 当前设定拉伸量(mm)
  bool charge_calibrated;

  /* ---- Yaw: 1 x M2006 ---- */
  DJIMotorInstance* aim_motor;
  float aim_zero;           // 找零得到的零点(转子角度)
  float aim_target_angle;   // 目标角度(转子角度)
  bool aim_calibrated;

  /* ---- 扳机: PWM 舵机 ---- */
  PWMInstance* trigger_pwm;
  bool trigger_released;

  /* ---- 状态 ---- */
  LauncherMode_e mode;
  LauncherCaliStep_e cali_step;
  LauncherAutoStep_e auto_step;
  bool calibrated;          // 蓄力与 yaw 均已校准
  bool estop;
  RC_ctrl_t* rc;
  float target_energy_j;    // 目标发射能量
} LauncherInstance;

extern LauncherInstance* launcher;

void LauncherInit(void);
void LauncherTask(void);

#endif  // DART_LAUNCHER_LAUNCHER_H
