/*
 * launcher_charge.c — 蓄力机构 (2 x M3508 同步带拉簧)
 *
 * 两个 M3508 通过同步带刚性耦合, 用同一位置目标驱动; 由于其中一个可能镜像
 * 安装(反馈反向), 统一以"机械同向增量"为输入, 内部按各自零点与符号换算。
 */
#include "launcher_charge.h"

#include <math.h>

#include "robot_config.h"
#include "user_lib.h"

/* 电机反向时, 机械同向对应的角度增量符号取反 */
#define CHARGE_MASTER_SIGN (((CHARGE_MASTER_REVERSE) == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f)
#define CHARGE_SLAVE_SIGN  (((CHARGE_SLAVE_REVERSE) == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f)

/* 读取并清除速度环堵转标志 (校准找限位用) */
static bool MotorBlocked(DJIMotorInstance* motor) {
  if (motor == NULL) return false;
  if (motor->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
    motor->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
    motor->motor_controller.speed_PID.ERRORHandler.ERRORCount = 0;
    return true;
  }
  return false;
}

static void ClearSpeedPID(DJIMotorInstance* motor) {
  if (motor == NULL) return;
  motor->motor_controller.speed_PID.ITerm = 0;
  motor->motor_controller.speed_PID.Output = 0;
  motor->motor_controller.speed_PID.Iout = 0;
  motor->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
  motor->motor_controller.speed_PID.ERRORHandler.ERRORCount = 0;
}

void LauncherChargeInit(LauncherInstance* l) {
  Motor_Init_Config_s master_cfg =
      CHARGE_MOTOR_CONFIG(LAUNCHER_CAN_HANDLE, CHARGE_MASTER_ID, CHARGE_MASTER_REVERSE);
  l->charge_master = DJIMotorInit(&master_cfg);

  Motor_Init_Config_s slave_cfg =
      CHARGE_MOTOR_CONFIG(LAUNCHER_CAN_HANDLE, CHARGE_SLAVE_ID, CHARGE_SLAVE_REVERSE);
  l->charge_slave = DJIMotorInit(&slave_cfg);

  l->charge_zero_master = 0.0f;
  l->charge_zero_slave = 0.0f;
  l->charge_target_delta = 0.0f;
  l->charge_draw_mm = 0.0f;
  l->charge_calibrated = false;

  LauncherChargeHold(l);
}

void LauncherChargeSetSpeed(LauncherInstance* l, float speed) {
  DJIMotorOuterLoop(l->charge_master, SPEED_LOOP);
  DJIMotorSetPIDRef(l->charge_master, CHARGE_MASTER_SIGN * speed);
  DJIMotorOuterLoop(l->charge_slave, SPEED_LOOP);
  DJIMotorSetPIDRef(l->charge_slave, CHARGE_SLAVE_SIGN * speed);
}

void LauncherChargeSetDelta(LauncherInstance* l, float delta) {
  l->charge_target_delta = delta;
  DJIMotorOuterLoop(l->charge_master, ANGLE_LOOP);
  DJIMotorSetPIDRef(l->charge_master, l->charge_zero_master + CHARGE_MASTER_SIGN * delta);
  DJIMotorOuterLoop(l->charge_slave, ANGLE_LOOP);
  DJIMotorSetPIDRef(l->charge_slave, l->charge_zero_slave + CHARGE_SLAVE_SIGN * delta);
}

void LauncherChargeSetDrawMM(LauncherInstance* l, float mm) {
  VAL_LIMIT(mm, CHARGE_MIN_DRAW_MM, CHARGE_MAX_DRAW_MM);
  l->charge_draw_mm = mm;
  LauncherChargeSetDelta(l, mm * CHARGE_DEG_PER_MM);
}

void LauncherChargeSetEnergy(LauncherInstance* l, float energy_j) {
  /* E = F0*x + 0.5*k*x^2  =>  x = (-F0 + sqrt(F0^2 + 2*k*E)) / k */
  const float k = SPRING_K_TOTAL_N_PER_M;
  const float f0 = SPRING_PRELOAD_N;
  if (energy_j < 0.0f) energy_j = 0.0f;
  float x = (-f0 + sqrtf(f0 * f0 + 2.0f * k * energy_j)) / k;  // 单位 m
  LauncherChargeSetDrawMM(l, x * 1000.0f);                    // m -> mm
}

void LauncherChargeHold(LauncherInstance* l) {
  LauncherChargeSetDelta(l, l->charge_target_delta);
}

void LauncherChargeStop(LauncherInstance* l) {
  DJIMotorStop(l->charge_master);
  DJIMotorStop(l->charge_slave);
  ClearSpeedPID(l->charge_master);
  ClearSpeedPID(l->charge_slave);
}

bool LauncherChargeBlocked(LauncherInstance* l) {
  bool blocked = MotorBlocked(l->charge_master) | MotorBlocked(l->charge_slave);
  return blocked;
}

bool LauncherChargeInPosition(LauncherInstance* l) {
  float target_m = l->charge_zero_master + CHARGE_MASTER_SIGN * l->charge_target_delta;
  bool pos = (fabsf(l->charge_master->measure.total_angle - target_m) < CALI_POS_THRESHOLD) &&
             (fabsf(l->charge_master->measure.speed_aps) < CALI_SPEED_THRESHOLD);
  return pos;
}

float LauncherChargeDrawMM(LauncherInstance* l) {
  float delta = (l->charge_master->measure.total_angle - l->charge_zero_master) / CHARGE_MASTER_SIGN;
  return delta / CHARGE_DEG_PER_MM;
}
