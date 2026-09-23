/*
 * launcher_aim.c — Yaw 瞄准机构 (1 x M2006 + 丝杆平移前轴承)
 *
 * M2006 转子角度经丝杆 -> 前轴承平移 -> 射出方向 yaw 的换算系数为
 * AIM_DEG_PER_DEG (需实测标定), 这里统一以转子角度闭环。
 */
#include "launcher_aim.h"

#include <math.h>

#include "robot_config.h"
#include "user_lib.h"

#define AIM_SIGN (((AIM_MOTOR_REVERSE) == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f)

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

void LauncherAimInit(LauncherInstance* l) {
  Motor_Init_Config_s cfg = AIM_MOTOR_CONFIG(LAUNCHER_CAN_HANDLE, AIM_MOTOR_ID, AIM_MOTOR_REVERSE);
  l->aim_motor = DJIMotorInit(&cfg);

  l->aim_zero = 0.0f;
  l->aim_target_angle = 0.0f;
  l->aim_calibrated = false;

  LauncherAimHold(l);
}

void LauncherAimSetSpeed(LauncherInstance* l, float speed) {
  DJIMotorOuterLoop(l->aim_motor, SPEED_LOOP);
  DJIMotorSetPIDRef(l->aim_motor, AIM_SIGN * speed);
}

void LauncherAimSetAngle(LauncherInstance* l, float angle) {
  l->aim_target_angle = angle;
  DJIMotorOuterLoop(l->aim_motor, ANGLE_LOOP);
  DJIMotorSetPIDRef(l->aim_motor, angle);
}

void LauncherAimSetYawDeg(LauncherInstance* l, float yaw_deg) {
  VAL_LIMIT(yaw_deg, AIM_MIN_DEG, AIM_MAX_DEG);
  float delta = AIM_SIGN * yaw_deg / AIM_DEG_PER_DEG;  // 机械同向 -> 反馈角度增量
  LauncherAimSetAngle(l, l->aim_zero + delta);
}

void LauncherAimHold(LauncherInstance* l) {
  LauncherAimSetAngle(l, l->aim_target_angle);
}

void LauncherAimStop(LauncherInstance* l) {
  DJIMotorStop(l->aim_motor);
  ClearSpeedPID(l->aim_motor);
}

bool LauncherAimBlocked(LauncherInstance* l) {
  return MotorBlocked(l->aim_motor);
}

bool LauncherAimInPosition(LauncherInstance* l) {
  return (fabsf(l->aim_motor->measure.total_angle - l->aim_target_angle) < CALI_POS_THRESHOLD) &&
         (fabsf(l->aim_motor->measure.speed_aps) < CALI_SPEED_THRESHOLD);
}
