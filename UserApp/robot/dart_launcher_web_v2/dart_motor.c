/*
 * dart_motor.c — 四电机驱动: 速度/角度/圈数三模式 + 每电机独立参数/零点/方向
 */
#include "dart_motor.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

DartParam_s g_dart_params[DART_MOTOR_COUNT];
DartCtrl_t g_dart_ctrl[DART_MOTOR_COUNT];
volatile uint8_t g_dart_scanning = 0;

extern int g_estop;  // dart_fsm.c

static const SimpleMotorType_e kType[DART_MOTOR_COUNT] = {
    DART_SPRING_A_TYPE, DART_SPRING_B_TYPE, DART_TRIGGER_TYPE, DART_YAW_TYPE};
static const uint8_t kId[DART_MOTOR_COUNT] = {DART_SPRING_A_ID, DART_SPRING_B_ID, DART_TRIGGER_ID,
                                              DART_YAW_ID};
static const uint8_t kReverse[DART_MOTOR_COUNT] = {DART_SPRING_A_REVERSE, DART_SPRING_B_REVERSE,
                                                   DART_TRIGGER_REVERSE, DART_YAW_REVERSE};

static const DartParam_s kDefaultM3508 = {
    1.5f, 6.0f, 1.5f, 2000.0f, 2.0f, 0.1f, 800.0f, DART_M3508_GEAR_RATIO, 2.0f, 80.0f, 6000};
static const DartParam_s kDefaultM2006 = {
    2.0f, 8.0f, 2.0f, 2000.0f, 2.5f, 0.1f, 800.0f, DART_M2006_GEAR_RATIO, 1.0f, 120.0f, 5000};

static uint32_t s_prev_fb[DART_MOTOR_COUNT];
static uint32_t s_online_tick;

static float Dir(int s) { return g_dart_ctrl[s].reverse ? -1.0f : 1.0f; }

void DartMotorResetParams(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  g_dart_params[slot] = (kType[slot] == SIMPLE_MOTOR_M2006) ? kDefaultM2006 : kDefaultM3508;
}

void DartMotorLoadParams(int slot, const DartParam_s* p) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT || p == NULL) return;
  g_dart_params[slot] = *p;
}

void DartMotorLoadZero(int slot, float zero, uint8_t valid) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  g_dart_ctrl[slot].zero = zero;
  g_dart_ctrl[slot].zero_valid = valid;
}

void DartMotorInit(void) {
  memset(g_dart_params, 0, sizeof(g_dart_params));
  memset(g_dart_ctrl, 0, sizeof(g_dart_ctrl));

  SimpleMotorBusInit();
  for (int i = 0; i < DART_MOTOR_COUNT; i++) SimpleMotorAdd(kType[i], kId[i]);

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    DartMotorResetParams(i);
    g_dart_ctrl[i].mode = DART_MODE_STOP;
    g_dart_ctrl[i].reverse = kReverse[i];
  }
  SimpleMotorSendAll();
}

void DartMotorSetCtrl(int slot, int mode, float value) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  DartCtrl_t* c = &g_dart_ctrl[slot];
  switch (mode) {
    case DART_MODE_STOP:
      c->mode = DART_MODE_STOP;
      c->target = 0.0f;
      c->integral = 0.0f;
      c->speed_cmd = 0.0f;
      c->stalled = 0;
      break;
    case DART_MODE_SPEED:
    case DART_MODE_ANGLE:
    case DART_MODE_TURNS:
      c->mode = (DartMode_e)mode;
      c->target = value;
      c->integral = 0.0f;
      c->speed_cmd = 0.0f;
      c->run_ms = 0;
      c->stall_ms = 0;
      c->stalled = 0;
      break;
    default:
      break;
  }
}

void DartMotorStopAll(void) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++) DartMotorSetCtrl(i, DART_MODE_STOP, 0.0f);
}

void DartMotorZero(int slot) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    if (slot >= 0 && i != slot) continue;
    g_dart_ctrl[i].zero = Dir(i) * g_motors[i].total_angle;
    g_dart_ctrl[i].zero_valid = 1;
    if (g_dart_ctrl[i].mode != DART_MODE_STOP) g_dart_ctrl[i].target = 0.0f;
  }
}

void DartMotorToggleDir(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  g_dart_ctrl[slot].reverse = g_dart_ctrl[slot].reverse ? 0 : 1;
  /* 方向变了, 零点参考随之取反 */
  g_dart_ctrl[slot].zero = Dir(slot) * g_motors[slot].total_angle;
  g_dart_ctrl[slot].zero_valid = 1;
}

void DartMotorSetParam(int slot, int id, int value) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  DartParam_s* p = &g_dart_params[slot];
  switch (id) {
    case 1: p->speed_ff = value / 100.0f; break;
    case 2: p->speed_kp = value / 100.0f; break;
    case 3: p->speed_ki = value / 100.0f; break;
    case 4: p->speed_i_limit = value / 100.0f; break;
    case 5: p->angle_kp = value / 100.0f; break;
    case 7: p->angle_kd = value / 100.0f; break;
    case 8: p->angle_slew = value / 100.0f; break;
    case 10: p->gear_ratio = value / 100.0f; break;
    case 11: p->angle_deadband = value / 100.0f; break;
    case 12: p->angle_speed_max = value / 100.0f; break;
    case 6:
      if (value < 0) value = 0;
      if (value > 30000) value = 30000;
      p->max_value = value;
      break;
    default: return;
  }
}

float DartMotorOutAngle(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return 0.0f;
  float ratio = (g_dart_params[slot].gear_ratio > 0.01f) ? g_dart_params[slot].gear_ratio : 1.0f;
  return (Dir(slot) * g_motors[slot].total_angle - g_dart_ctrl[slot].zero) / ratio;
}

float DartMotorOutTurns(int slot) { return DartMotorOutAngle(slot) / 360.0f; }

int DartMotorAtTurns(int slot, float turns, float tol) {
  return fabsf(DartMotorOutTurns(slot) - turns) < tol;
}

/* ===================== 任务 ===================== */
void DartMotorTask(void) {
  if (!g_dart_scanning) SimpleMotorReadAll();

  if (g_estop) {
    for (int i = 0; i < DART_MOTOR_COUNT; i++) {
      g_dart_ctrl[i].integral = 0.0f;
      g_dart_ctrl[i].speed_cmd = 0.0f;
      SimpleMotorSet(i, 0);
    }
    SimpleMotorSendAll();
    return;
  }

  if (HAL_GetTick() - s_online_tick >= 100u) {
    s_online_tick = HAL_GetTick();
    for (int i = 0; i < DART_MOTOR_COUNT; i++) {
      g_dart_ctrl[i].online = (g_motors[i].fb_count != s_prev_fb[i]) ? 1 : 0;
      s_prev_fb[i] = g_motors[i].fb_count;
    }
  }

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    SimpleMotor_t* m = &g_motors[i];
    DartParam_s* p = &g_dart_params[i];
    DartCtrl_t* c = &g_dart_ctrl[i];
    float dir = Dir(i);
    float ratio = (p->gear_ratio > 0.01f) ? p->gear_ratio : 1.0f;
    float fb = dir * (float)m->rpm;  // 转子 rpm (方向修正)

    if (m->inited && !c->zero_valid) {
      c->zero = dir * m->total_angle;
      c->zero_valid = 1;
    }

    if (c->mode == DART_MODE_STOP) {
      c->integral = 0.0f;
      c->speed_cmd = 0.0f;
      c->stalled = 0;
      SimpleMotorSet(i, 0);
      continue;
    }

    c->run_ms++;

    /* 目标速度 (输出 rpm) */
    float spd_out;
    if (c->mode == DART_MODE_SPEED) {
      spd_out = c->target;
    } else {
      float tgt_out = (c->mode == DART_MODE_TURNS) ? c->target * 360.0f : c->target;
      float cur_out = (dir * m->total_angle - c->zero) / ratio;
      float err = tgt_out - cur_out;
      if (fabsf(err) < p->angle_deadband) {
        spd_out = 0.0f;
      } else {
        float rate = fb / ratio / 60.0f * 360.0f;  // 输出 deg/s
        spd_out = p->angle_kp * err - p->angle_kd * rate;
        VAL_LIMIT(spd_out, -p->angle_speed_max, p->angle_speed_max);
      }
      if (p->angle_slew > 0.01f) {
        float maxd = p->angle_slew * DART_CTRL_DT;
        float d = spd_out - c->speed_cmd;
        if (d > maxd) spd_out = c->speed_cmd + maxd;
        else if (d < -maxd) spd_out = c->speed_cmd - maxd;
      }
    }
    c->speed_cmd = spd_out;

    /* 内环速度 PI + 前馈 */
    float tgt_motor = spd_out * ratio;
    float err = tgt_motor - fb;
    c->integral += err * DART_CTRL_DT;
    VAL_LIMIT(c->integral, -p->speed_i_limit, p->speed_i_limit);
    float out = p->speed_ff * tgt_motor + p->speed_kp * err + p->speed_ki * c->integral;
    VAL_LIMIT(out, -(float)p->max_value, (float)p->max_value);

    /* 堵转保护 */
    if (c->run_ms > (DART_STALL_GRACE_MS)) {
      if (fabsf((float)m->current) > DART_STALL_CUR && fabsf((float)m->rpm) < 15.0f) {
        c->stall_ms++;
      } else {
        c->stall_ms = 0;
      }
      if (c->stall_ms > DART_STALL_MS) {
        c->mode = DART_MODE_STOP;
        c->stalled = 1;
        out = 0.0f;
      }
    }

    SimpleMotorSet(i, (int16_t)(dir * out));
  }

  SimpleMotorSendAll();
}
