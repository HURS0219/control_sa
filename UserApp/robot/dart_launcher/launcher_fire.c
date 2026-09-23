/*
 * launcher_fire.c — 扳机释放 (PWM 舵机, 50Hz)
 *
 * 报告方案: 蓄力到位后舵机拨动扳机释放拉簧能量。
 * F407 GIMBAL_BOARD: PE9 = TIM1_CH1, 已配置 50Hz (PSC=167, ARR=19999)。
 */
#include "launcher_fire.h"

#include "bsp_pwm.h"
#include "robot_config.h"
#include "tim.h"

static float PulseToDuty(float pulse_us) {
  return pulse_us / (TRIGGER_PWM_PERIOD_S * 1000000.0f);
}

void LauncherTriggerInit(LauncherInstance* l) {
  PWM_Init_Config_s cfg = {
      .htim = TRIGGER_PWM_HANDLE,
      .channel = TRIGGER_PWM_CHANNEL,
      .period = TRIGGER_PWM_PERIOD_S,
      .dutyratio = PulseToDuty(TRIGGER_LOCK_US),
      .callback = NULL,
      .id = NULL,
  };
  l->trigger_pwm = PWMRegister(&cfg);
  l->trigger_released = false;
  LauncherTriggerLock(l);
}

void LauncherTriggerRelease(LauncherInstance* l) {
  if (l->trigger_pwm == NULL) return;
  PWMSetDutyRatio(l->trigger_pwm, PulseToDuty(TRIGGER_RELEASE_US));
  l->trigger_released = true;
}

void LauncherTriggerLock(LauncherInstance* l) {
  if (l->trigger_pwm == NULL) return;
  PWMSetDutyRatio(l->trigger_pwm, PulseToDuty(TRIGGER_LOCK_US));
  l->trigger_released = false;
}
