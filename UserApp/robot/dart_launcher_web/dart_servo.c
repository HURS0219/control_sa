/*
 * dart_servo.c — 扳机舵机 PWM 输出
 */
#include "dart_servo.h"

#include "bsp_pwm.h"
#include "robot_config.h"
#include "user_lib.h"

float g_servo_std_us = DART_SERVO_STD_US;
float g_servo_prep_us = DART_SERVO_PREP_US;
float g_servo_cur_us = DART_SERVO_STD_US;
int g_servo_state = DART_SERVO_STD;

static PWMInstance* s_pwm = NULL;

static float ClampUs(float us) {
  VAL_LIMIT(us, DART_SERVO_MIN_US, DART_SERVO_MAX_US);
  return us;
}

void DartServoInit(void) {
  PWM_Init_Config_s cfg = {
      .htim = DART_SERVO_TIM,
      .channel = DART_SERVO_CHANNEL,
      .period = DART_SERVO_PERIOD_S,
      .dutyratio = 0.0f,
      .callback = NULL,
      .id = NULL,
  };
  s_pwm = PWMRegister(&cfg);
  DartServoGo(DART_SERVO_STD);
}

void DartServoSetUs(float us) {
  if (s_pwm == NULL) return;
  g_servo_cur_us = ClampUs(us);
  PWMSetDutyRatio(s_pwm, g_servo_cur_us / (DART_SERVO_PERIOD_S * 1000000.0f));
}

void DartServoSetPos(int which, float us) {
  us = ClampUs(us);
  if (which == DART_SERVO_PREP) {
    g_servo_prep_us = us;
    if (g_servo_state == DART_SERVO_PREP) DartServoSetUs(us);
  } else {
    g_servo_std_us = us;
    if (g_servo_state == DART_SERVO_STD) DartServoSetUs(us);
  }
}

void DartServoGo(int which) {
  g_servo_state = (which == DART_SERVO_PREP) ? DART_SERVO_PREP : DART_SERVO_STD;
  DartServoSetUs(g_servo_state == DART_SERVO_PREP ? g_servo_prep_us : g_servo_std_us);
}

void DartServoTask(void) {}
