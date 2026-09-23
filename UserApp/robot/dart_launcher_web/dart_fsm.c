/*
 * dart_fsm.c — 发射架抽象状态机 + 自动化流程
 */
#include "dart_fsm.h"

#include <math.h>

#include "dart_motor.h"
#include "dart_servo.h"
#include "dart_vision.h"
#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

float g_spring_turns = DART_SPRING_PREP_TURNS;
int g_yaw_mode = DART_YAW_MANUAL;
int g_estop = 0;
int g_auto_step = DART_AUTO_IDLE;

#define SERVO_SETTLE_MS 400u

static void SpringSetTurns(float turns) {
  float deg = turns * 360.0f;
  DartMotorSetCtrl(DART_ROLE_SPRING_A, DART_CTRL_ANGLE, deg);
  DartMotorSetCtrl(DART_ROLE_SPRING_B, DART_CTRL_ANGLE, deg);
}

static int SpringAtTurns(float turns) {
  return DartMotorAtTargetTurns(DART_ROLE_SPRING_A, turns, DART_SPRING_POS_TOL) &&
         DartMotorAtTargetTurns(DART_ROLE_SPRING_B, turns, DART_SPRING_POS_TOL);
}

void DartFsmInit(void) {
  g_spring_turns = DART_SPRING_PREP_TURNS;
  g_yaw_mode = DART_YAW_MANUAL;
  g_estop = 0;
  g_auto_step = DART_AUTO_IDLE;
}

void DartFsmServoGo(int which) { DartServoGo(which); }

void DartFsmSpringGo(int which) {
  SpringSetTurns(which == 1 ? g_spring_turns : 0.0f);
}

void DartFsmAutoStart(void) {
  if (g_estop) return;
  g_auto_step = DART_AUTO_SERVO_STD1;
}

void DartFsmAutoStop(void) { g_auto_step = DART_AUTO_IDLE; }

void DartFsmSetEstop(int on) {
  g_estop = on ? 1 : 0;
  if (g_estop) {
    DartMotorStopAll();
    DartServoGo(DART_SERVO_STD);
    g_auto_step = DART_AUTO_IDLE;
  }
}

/* ---------------- Yaw ---------------- */
static void YawHandler(void) {
  if (g_yaw_mode == DART_YAW_VISION) {
    if (g_vis_ok) {
      float err = (float)g_vis_err;
      if (fabsf(err) < DART_YAW_AIM_DEADBAND) {
        DartMotorSetCtrl(DART_ROLE_YAW, DART_CTRL_SPEED, 0.0f);
      } else {
        float spd = DART_YAW_AIM_KP * err;
        VAL_LIMIT(spd, -g_dart_params[DART_ROLE_YAW].angle_speed_max,
                  g_dart_params[DART_ROLE_YAW].angle_speed_max);
        DartMotorSetCtrl(DART_ROLE_YAW, DART_CTRL_SPEED, spd);
      }
    } else {
      DartMotorSetCtrl(DART_ROLE_YAW, DART_CTRL_SPEED, 0.0f);
    }
  } else if (g_yaw_mode == DART_YAW_GUIDE) {
    DartMotorSetCtrl(DART_ROLE_YAW, DART_CTRL_ANGLE, DART_YAW_GUIDE_DEG);
  }
  /* MANUAL: 交给网页直接控制 */
}

/* ---------------- 自动化时序 ---------------- */
static void AutoHandler(void) {
  static uint32_t t0 = 0;
  uint32_t now = HAL_GetTick();

  switch (g_auto_step) {
    case DART_AUTO_IDLE:
    case DART_AUTO_DONE:
      break;

    case DART_AUTO_SERVO_STD1:
      DartServoGo(DART_SERVO_STD);
      t0 = now;
      g_auto_step = DART_AUTO_SPRING_STD1;
      break;

    case DART_AUTO_SPRING_STD1:
      SpringSetTurns(0.0f);
      if (SpringAtTurns(0.0f) || now - t0 > DART_FSM_STEP_TIMEOUT_MS) {
        t0 = now;
        g_auto_step = DART_AUTO_SPRING_PREP;
      }
      break;

    case DART_AUTO_SPRING_PREP:  // 拉簧到预备位 (蓄力)
      SpringSetTurns(g_spring_turns);
      if (SpringAtTurns(g_spring_turns) || now - t0 > DART_FSM_STEP_TIMEOUT_MS) {
        t0 = now;
        g_auto_step = DART_AUTO_SERVO_PREP;
      }
      break;

    case DART_AUTO_SERVO_PREP:  // 舵机预备位扣住发射台
      DartServoGo(DART_SERVO_PREP);
      if (now - t0 > SERVO_SETTLE_MS) {
        t0 = now;
        g_auto_step = DART_AUTO_SPRING_BACK;
      }
      break;

    case DART_AUTO_SPRING_BACK:  // 拉簧反转回标准位
      SpringSetTurns(0.0f);
      if (SpringAtTurns(0.0f) || now - t0 > DART_FSM_STEP_TIMEOUT_MS) {
        t0 = now;
        g_auto_step = DART_AUTO_SERVO_STD2;
      }
      break;

    case DART_AUTO_SERVO_STD2:  // 舵机复位 -> 释放发射台
      DartServoGo(DART_SERVO_STD);
      if (now - t0 > SERVO_SETTLE_MS) {
        g_auto_step = DART_AUTO_DONE;
      }
      break;

    default:
      g_auto_step = DART_AUTO_IDLE;
      break;
  }
}

void DartFsmTask(void) {
  if (g_estop) {
    DartMotorStopAll();
    return;
  }
  YawHandler();
  AutoHandler();
}
