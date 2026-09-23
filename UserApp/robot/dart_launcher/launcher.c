/*
 * launcher.c — 飞镖发射架顶层状态机与任务调度
 *
 * 遥控器右侧拨杆 (switch_right):
 *   下档 -> DEBUG : 左摇杆竖=蓄力, 左摇杆横=Yaw, 右摇杆竖向上=释放扳机
 *   中档 -> READY : 未校准则 CALIB, 已校准则保持
 *   上档 -> AUTO  : 未校准则 CALIB, 已校准则 蓄力->发射 时序
 *
 * 急停: 遥控器离线或置位 estop 时所有电机断电、扳机锁止。
 */
#include "launcher.h"

#include "bsp_log.h"
#include "launcher_aim.h"
#include "launcher_charge.h"
#include "launcher_fire.h"
#include "robot.h"
#include "robot_config.h"
#include "user_lib.h"

LauncherInstance* launcher = NULL;

/* ===================== 工具 ===================== */
static float MapStick(int16_t value, float max_value) {
  if (abs(value) < RC_DEADZONE) return 0.0f;
  return ((float)value / 660.0f) * max_value;
}

/* ===================== 初始化 ===================== */
void LauncherInit(void) {
  launcher = (LauncherInstance*)zmalloc(sizeof(LauncherInstance));
  memset(launcher, 0, sizeof(LauncherInstance));

  launcher->rc = RemoteControlInit(&huart3);

  LauncherChargeInit(launcher);
  LauncherAimInit(launcher);
  LauncherTriggerInit(launcher);

  launcher->mode = LAUNCHER_MODE_DEBUG;
  launcher->cali_step = CALI_IDLE;
  launcher->auto_step = AUTO_IDLE;
  launcher->calibrated = false;
  launcher->estop = false;
  launcher->target_energy_j = CHARGE_DEFAULT_ENERGY_J;

  LOGINFO("[dart_launcher] init done");
}

/* ===================== 模式切换 ===================== */
static void LauncherStateUpdate(void) {
  RC_ctrl_t* rc = launcher->rc;

#if LAUNCHER_ESTOP_ON_RC_LOST
  if (rc == NULL || !RemoteControlIsOnline()) {
    launcher->estop = true;
  }
#endif

  if (launcher->estop) {
    launcher->mode = LAUNCHER_MODE_ESTOP;
    return;
  }
  if (rc == NULL) return;

  LauncherMode_e new_mode = launcher->mode;
  if (switch_is_down(rc->rc.switch_right)) {
    new_mode = LAUNCHER_MODE_DEBUG;
  } else if (switch_is_mid(rc->rc.switch_right)) {
    new_mode = launcher->calibrated ? LAUNCHER_MODE_READY : LAUNCHER_MODE_CALIB;
  } else if (switch_is_up(rc->rc.switch_right)) {
    new_mode = launcher->calibrated ? LAUNCHER_MODE_AUTO : LAUNCHER_MODE_CALIB;
  }

  if (new_mode != launcher->mode) {
    if (new_mode == LAUNCHER_MODE_CALIB) launcher->cali_step = CALI_IDLE;
    if (new_mode == LAUNCHER_MODE_AUTO) launcher->auto_step = AUTO_IDLE;
    launcher->mode = new_mode;
  }
}

/* ===================== DEBUG ===================== */
static void LauncherDebugHandler(void) {
  RC_ctrl_t* rc = launcher->rc;
  if (rc == NULL) return;

  /* 蓄力: 左摇杆竖直, 带软限位 */
  float charge_speed = MapStick(rc->rc.rocker_l1, CHARGE_STICK_MAX_SPEED);
  float draw = LauncherChargeDrawMM(launcher);
  if (charge_speed > 0.0f && draw >= CHARGE_MAX_DRAW_MM) charge_speed = 0.0f;
  if (charge_speed < 0.0f && draw <= CHARGE_MIN_DRAW_MM) charge_speed = 0.0f;
  LauncherChargeSetSpeed(launcher, charge_speed);

  /* Yaw: 左摇杆水平 */
  LauncherAimSetSpeed(launcher, MapStick(rc->rc.rocker_l_, AIM_STICK_MAX_SPEED));

  /* 扳机: 右摇杆竖直向上 -> 释放, 否则锁止 */
  if (rc->rc.rocker_r1 > RC_DEADZONE) {
    LauncherTriggerRelease(launcher);
  } else {
    LauncherTriggerLock(launcher);
  }
}

/* ===================== 校准 ===================== */
static void LauncherCalibHandler(void) {
  static uint32_t step_t0 = 0;

  switch (launcher->cali_step) {
    case CALI_IDLE:
      step_t0 = HAL_GetTick();
      launcher->cali_step = CALI_CHARGE_RETRACT;
      break;

    case CALI_CHARGE_RETRACT:  // 蓄力机构向限位方向找零
      LauncherAimStop(launcher);
      LauncherChargeSetSpeed(launcher, CHARGE_CALI_DIR * CHARGE_CALI_SPEED);
      if (LauncherChargeBlocked(launcher) || (HAL_GetTick() - step_t0 > CALI_TIMEOUT_MS)) {
        launcher->charge_zero_master = launcher->charge_master->measure.total_angle;
        launcher->charge_zero_slave = launcher->charge_slave->measure.total_angle;
        launcher->charge_target_delta = 0.0f;
        LauncherChargeHold(launcher);
        step_t0 = HAL_GetTick();
        launcher->cali_step = CALI_CHARGE_BACK;
      }
      break;

    case CALI_CHARGE_BACK:  // 回退消形变
      LauncherChargeSetDrawMM(launcher, CALI_BACK_MM);
      if (LauncherChargeInPosition(launcher) || (HAL_GetTick() - step_t0 > CALI_TIMEOUT_MS)) {
        launcher->charge_calibrated = true;
        step_t0 = HAL_GetTick();
        launcher->cali_step = CALI_AIM_RETRACT;
      }
      break;

    case CALI_AIM_RETRACT:  // yaw 找限位
      LauncherChargeHold(launcher);
      LauncherAimSetSpeed(launcher, AIM_CALI_DIR * AIM_CALI_SPEED);
      if (LauncherAimBlocked(launcher) || (HAL_GetTick() - step_t0 > CALI_TIMEOUT_MS)) {
        launcher->aim_zero = launcher->aim_motor->measure.total_angle;
        LauncherAimHold(launcher);
        step_t0 = HAL_GetTick();
        launcher->cali_step = CALI_AIM_BACK;
      }
      break;

    case CALI_AIM_BACK:  // 回退到中位
      LauncherAimSetAngle(launcher, launcher->aim_zero);
      if (LauncherAimInPosition(launcher) || (HAL_GetTick() - step_t0 > CALI_TIMEOUT_MS)) {
        launcher->aim_calibrated = true;
        launcher->cali_step = CALI_DONE;
      }
      break;

    case CALI_DONE:
      launcher->calibrated = launcher->charge_calibrated && launcher->aim_calibrated;
      launcher->cali_step = CALI_IDLE;
      launcher->mode = LAUNCHER_MODE_READY;
      LOGINFO("[dart_launcher] calibrated=%d", launcher->calibrated);
      break;

    default:
      launcher->cali_step = CALI_IDLE;
      break;
  }
}

/* ===================== 自动发射 ===================== */
static void LauncherAutoHandler(void) {
  static uint32_t t0 = 0;

  switch (launcher->auto_step) {
    case AUTO_IDLE:
      LauncherChargeSetEnergy(launcher, launcher->target_energy_j);
      t0 = HAL_GetTick();
      launcher->auto_step = AUTO_CHARGING;
      break;

    case AUTO_CHARGING:  // 蓄力到目标能量
      LauncherChargeSetEnergy(launcher, launcher->target_energy_j);
      if (LauncherChargeInPosition(launcher) || (HAL_GetTick() - t0 > CALI_TIMEOUT_MS)) {
        t0 = HAL_GetTick();
        launcher->auto_step = AUTO_CHARGED;
      }
      break;

    case AUTO_CHARGED:  // 稳定后释放
      LauncherChargeHold(launcher);
      if (HAL_GetTick() - t0 > AUTO_CHARGE_SETTLE_MS) {
        LauncherTriggerRelease(launcher);
        t0 = HAL_GetTick();
        launcher->auto_step = AUTO_FIRING;
      }
      break;

    case AUTO_FIRING:  // 释放保持后锁止
      LauncherChargeHold(launcher);
      if (HAL_GetTick() - t0 > TRIGGER_RELEASE_HOLD_MS) {
        LauncherTriggerLock(launcher);
        t0 = HAL_GetTick();
        launcher->auto_step = AUTO_DONE;
      }
      break;

    case AUTO_DONE:  // 完成, 保持蓄力位等待操作手切换
      LauncherChargeHold(launcher);
      LauncherAimHold(launcher);
      break;

    default:
      launcher->auto_step = AUTO_IDLE;
      break;
  }
}

/* ===================== 急停 ===================== */
static void LauncherEstopHandler(void) {
  LauncherChargeStop(launcher);
  LauncherAimStop(launcher);
  LauncherTriggerLock(launcher);
}

/* ===================== 主任务 ===================== */
void LauncherTask(void) {
  LauncherStateUpdate();

  switch (launcher->mode) {
    case LAUNCHER_MODE_DEBUG:
      LauncherDebugHandler();
      break;

    case LAUNCHER_MODE_CALIB:
      LauncherCalibHandler();
      break;

    case LAUNCHER_MODE_READY:
      LauncherChargeHold(launcher);
      LauncherAimHold(launcher);
      LauncherTriggerLock(launcher);
      break;

    case LAUNCHER_MODE_AUTO:
      LauncherAutoHandler();
      break;

    case LAUNCHER_MODE_ESTOP:
      LauncherEstopHandler();
      break;

    default:
      break;
  }
}
