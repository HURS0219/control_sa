/*
 * dart_motor.c — 四电机角色抽象 + 每电机独立 PID 参数
 */
#include "dart_motor.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

DartParam_s g_dart_params[DART_MOTOR_COUNT];
DartCtrl_t g_dart_ctrl[DART_MOTOR_COUNT];
int g_dart_sel = 0;
volatile uint8_t g_dart_scanning = 0;

extern int g_estop;  // 定义于 dart_fsm.c; 急停时电机强制失能

/* 角色 -> simple_motor 槽位 (初始化时按角色顺序添加, 故一一对应) */
static const SimpleMotorType_e kType[DART_MOTOR_COUNT] = {
    DART_SPRING_A_TYPE, DART_SPRING_B_TYPE, DART_TRIGGER_TYPE, DART_YAW_TYPE};
static const uint8_t kId[DART_MOTOR_COUNT] = {DART_SPRING_A_ID, DART_SPRING_B_ID,
                                              DART_TRIGGER_ID, DART_YAW_ID};
static const uint8_t kReverse[DART_MOTOR_COUNT] = {DART_SPRING_A_REVERSE, DART_SPRING_B_REVERSE,
                                                   DART_TRIGGER_REVERSE, DART_YAW_REVERSE};

/* 出厂默认参数: 3508 / 2006 两套 */
static const DartParam_s kDefaultM3508 = {
    /*ff*/ 2.0f, /*kp*/ 8.0f, /*ki*/ 2.0f, /*ilim*/ 3000.0f,
    /*akp*/ 5.0f, /*akd*/ 0.0f, /*aslew*/ 0.0f, /*ratio*/ DART_M3508_GEAR_RATIO,
    /*dead*/ 10.0f, /*avmax*/ 150.0f, /*max*/ 6000};
static const DartParam_s kDefaultM2006 = {
    /*ff*/ 3.0f, /*kp*/ 10.0f, /*ki*/ 2.0f, /*ilim*/ 3000.0f,
    /*akp*/ 6.0f, /*akd*/ 0.0f, /*aslew*/ 0.0f, /*ratio*/ DART_M2006_GEAR_RATIO,
    /*dead*/ 1.0f, /*avmax*/ 120.0f, /*max*/ 5000};

static uint32_t s_prev_fb[DART_MOTOR_COUNT];
static uint32_t s_last_online_check;

static float Dir(int slot) { return g_dart_ctrl[slot].reverse ? -1.0f : 1.0f; }

void DartMotorInit(void) {
  memset(g_dart_params, 0, sizeof(g_dart_params));
  memset(g_dart_ctrl, 0, sizeof(g_dart_ctrl));

  SimpleMotorBusInit();
  for (int i = 0; i < DART_MOTOR_COUNT; i++) SimpleMotorAdd(kType[i], kId[i]);

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    g_dart_ctrl[i].mode = DART_CTRL_STOP;
    g_dart_ctrl[i].target = 0.0f;
    g_dart_ctrl[i].integral = 0.0f;
    g_dart_ctrl[i].zero = 0.0f;
    g_dart_ctrl[i].zero_valid = 0;
    g_dart_ctrl[i].online = 0;
    g_dart_ctrl[i].reverse = kReverse[i];
    DartMotorResetParams(i);
  }
  g_dart_sel = 0;
  SimpleMotorSendAll();
}

void DartMotorResetParams(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  g_dart_params[slot] = (kType[slot] == SIMPLE_MOTOR_M2006) ? kDefaultM2006 : kDefaultM3508;
}

int DartMotorSlotOfRole(DartRole_e role) { return (int)role; }

void DartMotorSelect(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  g_dart_sel = slot;
}

void DartMotorSetCtrl(int slot, int mode, float value) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  DartCtrl_t* c = &g_dart_ctrl[slot];
  DartParam_s* p = &g_dart_params[slot];

  switch (mode) {
    case DART_CTRL_STOP:
      c->mode = DART_CTRL_STOP;
      c->target = 0.0f;
      break;
    case DART_CTRL_SPEED:
      c->target = value;  // 输出轴 rpm
      c->mode = DART_CTRL_SPEED;
      break;
    case DART_CTRL_ANGLE:
      /* 角度零点由“设零”(Z) 或掉电保存给定, 不在进入时重置,
       * 这样状态机的 0 圈 / x 圈才是同一个绝对参考。 */
      c->target = value;  // 输出轴 deg
      c->mode = DART_CTRL_ANGLE;
      (void)p;
      break;
    default:
      break;
  }
}

void DartMotorStopAll(void) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    g_dart_ctrl[i].mode = DART_CTRL_STOP;
    g_dart_ctrl[i].target = 0.0f;
    g_dart_ctrl[i].integral = 0.0f;
  }
}

void DartMotorZero(int slot) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    if (slot >= 0 && i != slot) continue;
    g_dart_ctrl[i].zero = Dir(i) * g_motors[i].total_angle;
    g_dart_ctrl[i].zero_valid = 1;
    if (g_dart_ctrl[i].mode == DART_CTRL_ANGLE) g_dart_ctrl[i].target = 0.0f;
  }
}

int DartMotorAtTargetTurns(int slot, float turns, float tol) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return 0;
  SimpleMotor_t* m = &g_motors[slot];
  DartParam_s* p = &g_dart_params[slot];
  DartCtrl_t* c = &g_dart_ctrl[slot];
  float ratio = (p->gear_ratio > 0.01f) ? p->gear_ratio : 1.0f;
  float cur_turns = (Dir(slot) * m->total_angle - c->zero) / ratio / 360.0f;
  return fabsf(cur_turns - turns) < tol;
}

/* 参数设置: id 与 wifi_gm6020 一致, value 为浮点 *100 */
void DartMotorSetParam(int id, int value) {
  int slot = g_dart_sel;
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

void DartMotorStoreActive(void) {}
void DartMotorLoadActive(void) {}

/* ===================== 控制任务 ===================== */
void DartMotorTask(void) {
  if (!g_dart_scanning) SimpleMotorReadAll();

  if (g_estop) {
    for (int i = 0; i < DART_MOTOR_COUNT; i++) {
      g_dart_ctrl[i].integral = 0.0f;
      SimpleMotorSet(i, 0);
    }
    SimpleMotorSendAll();
    return;
  }

  /* 在线检测 */
  if (HAL_GetTick() - s_last_online_check >= 100u) {
    s_last_online_check = HAL_GetTick();
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
    float fb_rpm = dir * (float)m->rpm;

    /* 首次收到反馈且无保存零点: 以当前位置为 0 圈 */
    if (m->inited && !c->zero_valid) {
      c->zero = dir * m->total_angle;
      c->zero_valid = 1;
    }

    switch (c->mode) {
      case DART_CTRL_SPEED:
      case DART_CTRL_ANGLE: {
        float tgt_out;
        if (c->mode == DART_CTRL_ANGLE) {
          float rel_motor = dir * m->total_angle - c->zero;  // 转子 deg
          float err_out = c->target - rel_motor / ratio;      // 输出 deg
          if (fabsf(err_out) < p->angle_deadband) {
            tgt_out = 0.0f;
          } else {
            tgt_out = p->angle_kp * err_out;
            VAL_LIMIT(tgt_out, -p->angle_speed_max, p->angle_speed_max);
          }
        } else {
          tgt_out = c->target;
        }

        float tgt_motor = tgt_out * ratio;  // 转子 rpm
        float err = tgt_motor - fb_rpm;
        c->integral += err * DART_CTRL_DT;
        VAL_LIMIT(c->integral, -p->speed_i_limit, p->speed_i_limit);
        float out = p->speed_ff * tgt_motor + p->speed_kp * err + p->speed_ki * c->integral;
        VAL_LIMIT(out, -(float)p->max_value, (float)p->max_value);
        SimpleMotorSet(i, (int16_t)(dir * out));
        break;
      }
      case DART_CTRL_STOP:
      default:
        c->integral = 0.0f;
        SimpleMotorSet(i, 0);
        break;
    }
  }

  SimpleMotorSendAll();
}
