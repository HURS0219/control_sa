/*
 * dart_motor.c — 四电机封装 (完全基于 DJImotor 库)
 */
#include "dart_motor.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "robot_config.h"
#include "user_lib.h"

DartMotor_t g_dart_motors[DART_MOTOR_COUNT];

extern int g_estop;  // dart_fsm.c

/* 每个角色的默认减速比 */
static const float kRatio[DART_MOTOR_COUNT] = {
    DART_M3508_GEAR_RATIO, DART_M3508_GEAR_RATIO, DART_M3508_GEAR_RATIO, DART_M2006_GEAR_RATIO};

/* 出厂默认 PID (转子单位) */
typedef struct {
  float skp, ski, skd, silim, smax;
  float akp, aki, akd, adead, amax;
} DartPidDefault_s;

static const DartPidDefault_s kDefM3508 = {2.5f, 0.3f, 0.0f, 1500.0f, 16384.0f,
                                           6.0f, 0.0f, 0.25f, 2.0f, 3000.0f};
static const DartPidDefault_s kDefM2006 = {3.0f, 0.5f, 0.0f, 1500.0f, 10000.0f,
                                           6.0f, 0.0f, 0.25f, 1.0f, 4000.0f};

void DartMotorInit(void) {
  static Motor_Init_Config_s cfgs[DART_MOTOR_COUNT] = {
      DART_M3508_CONFIG(DART_SPRING_A_ID, DART_SPRING_A_REVERSE),
      DART_M3508_CONFIG(DART_SPRING_B_ID, DART_SPRING_B_REVERSE),
      DART_M3508_CONFIG(DART_TRIGGER_ID, DART_TRIGGER_REVERSE),
      DART_M2006_CONFIG(DART_YAW_ID, DART_YAW_REVERSE),
  };

  memset(g_dart_motors, 0, sizeof(g_dart_motors));
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    g_dart_motors[i].inst = DJIMotorInit(&cfgs[i]);
    g_dart_motors[i].ratio = kRatio[i];
    g_dart_motors[i].mode = DART_MODE_STOP;
    g_dart_motors[i].zero_valid = 0;
    DJIMotorStop(g_dart_motors[i].inst);
  }
}

static float SignOf(DartMotor_t* m) {
  return (m->inst->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
}

static float OutAngleOf(DartMotor_t* m) {
  /* DJI 库对反向电机的 total_angle 已取反, 换算回机械方向需乘 sign */
  return SignOf(m) * (m->inst->measure.total_angle - m->zero) / m->ratio;
}

float DartMotorOutAngle(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return 0.0f;
  return OutAngleOf(&g_dart_motors[slot]);
}
float DartMotorOutTurns(int slot) { return DartMotorOutAngle(slot) / 360.0f; }
int DartMotorAtTurns(int slot, float turns, float tol) {
  return fabsf(DartMotorOutTurns(slot) - turns) < tol;
}

void DartMotorSetCtrl(int slot, int mode, float value) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  DartMotor_t* m = &g_dart_motors[slot];
  m->mode = (DartMode_e)mode;
  m->target = value;
  if (mode == DART_MODE_STOP) DJIMotorStop(m->inst);
}

void DartMotorStopAll(void) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    g_dart_motors[i].mode = DART_MODE_STOP;
    g_dart_motors[i].target = 0.0f;
    DJIMotorStop(g_dart_motors[i].inst);
  }
}

void DartMotorZero(int slot) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    if (slot >= 0 && i != slot) continue;
    g_dart_motors[i].zero = g_dart_motors[i].inst->measure.total_angle;
    g_dart_motors[i].zero_valid = 1;
    if (g_dart_motors[i].mode != DART_MODE_STOP) g_dart_motors[i].target = 0.0f;
  }
}

void DartMotorToggleDir(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  DartMotor_t* m = &g_dart_motors[slot];
  Motor_Reverse_Flag_e r = (m->inst->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_NORMAL)
                               ? MOTOR_DIRECTION_REVERSE
                               : MOTOR_DIRECTION_NORMAL;
  m->inst->motor_settings.motor_reverse_flag = r;
  m->inst->motor_settings.feedback_reverse_flag = (r == MOTOR_DIRECTION_REVERSE)
                                                      ? FEEDBACK_DIRECTION_REVERSE
                                                      : FEEDBACK_DIRECTION_NORMAL;
  m->zero = m->inst->measure.total_angle;  // 方向变了重新取零点
  m->zero_valid = 1;
}

static DartPidDefault_s DefOf(int slot) {
  return (slot == DART_ROLE_YAW) ? kDefM2006 : kDefM3508;
}

void DartMotorLoadParamRaw(int slot, int id, float v) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  PIDInstance* sp = &g_dart_motors[slot].inst->motor_controller.speed_PID;
  PIDInstance* ap = &g_dart_motors[slot].inst->motor_controller.angle_PID;
  switch (id) {
    case 1: sp->Kp = v; break;
    case 2: sp->Ki = v; break;
    case 3: sp->Kd = v; break;
    case 4: sp->IntegralLimit = v; break;
    case 5: sp->MaxOut = v; break;
    case 6: ap->Kp = v; break;
    case 7: ap->Ki = v; break;
    case 8: ap->Kd = v; break;
    case 9: ap->DeadBand = v; break;
    case 10: ap->MaxOut = v; break;
    case 11:
      if (v > 0.01f) g_dart_motors[slot].ratio = v;
      break;
    default: break;
  }
}

void DartMotorSetParam(int slot, int id, int value) {
  DartMotorLoadParamRaw(slot, id, value / 100.0f);
}

void DartMotorReadParams(int slot, float* out) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT || out == NULL) return;
  PIDInstance* sp = &g_dart_motors[slot].inst->motor_controller.speed_PID;
  PIDInstance* ap = &g_dart_motors[slot].inst->motor_controller.angle_PID;
  out[0] = sp->Kp;
  out[1] = sp->Ki;
  out[2] = sp->Kd;
  out[3] = sp->IntegralLimit;
  out[4] = sp->MaxOut;
  out[5] = ap->Kp;
  out[6] = ap->Ki;
  out[7] = ap->Kd;
  out[8] = ap->DeadBand;
  out[9] = ap->MaxOut;
  out[10] = g_dart_motors[slot].ratio;
}

void DartMotorLoadZero(int slot, float zero, uint8_t valid) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  g_dart_motors[slot].zero = zero;
  g_dart_motors[slot].zero_valid = valid;
}

void DartMotorLoadDir(int slot, uint8_t reverse) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  Motor_Reverse_Flag_e r = reverse ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
  g_dart_motors[slot].inst->motor_settings.motor_reverse_flag = r;
  g_dart_motors[slot].inst->motor_settings.feedback_reverse_flag =
      reverse ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
}

void DartMotorResetParams(int slot) {
  if (slot < 0 || slot >= DART_MOTOR_COUNT) return;
  DartPidDefault_s d = DefOf(slot);
  DartMotorLoadParamRaw(slot, 1, d.skp);
  DartMotorLoadParamRaw(slot, 2, d.ski);
  DartMotorLoadParamRaw(slot, 3, d.skd);
  DartMotorLoadParamRaw(slot, 4, d.silim);
  DartMotorLoadParamRaw(slot, 5, d.smax);
  DartMotorLoadParamRaw(slot, 6, d.akp);
  DartMotorLoadParamRaw(slot, 7, d.aki);
  DartMotorLoadParamRaw(slot, 8, d.akd);
  DartMotorLoadParamRaw(slot, 9, d.adead);
  DartMotorLoadParamRaw(slot, 10, d.amax);
  DartMotorLoadParamRaw(slot, 11, kRatio[slot]);
}

void DartMotorTask(void) {
  /* DWT 兜底: 某些情况下 (如调试器断开) TRCENA 被清, 会导致 DJI PID 的 dt=0 */
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    DartMotor_t* m = &g_dart_motors[i];
    DJIMotorInstance* inst = m->inst;

    /* 在线检测 */
    if (inst->feed_cnt != m->last_feed) {
      m->last_feed = inst->feed_cnt;
      m->online = 1;
    } else {
      /* feed_cnt 由反馈中断递增; 超过阈值视为离线 (粗略, daemon 也有独立检测) */
    }

    /* 首次反馈取零点 */
    if (inst->feed_cnt > 0 && !m->zero_valid) {
      m->zero = inst->measure.total_angle;
      m->zero_valid = 1;
    }

    if (g_estop) {
      DJIMotorStop(inst);
      continue;
    }

    if (m->mode == DART_MODE_STOP) {
      DJIMotorStop(inst);
      continue;
    }

    DJIMotorEnable(inst);

    if (m->mode == DART_MODE_SPEED) {
      DJIMotorOuterLoop(inst, SPEED_LOOP);
      float ref = SignOf(m) * m->target * m->ratio * 6.0f;  // 机械输出 rpm -> 转子 deg/s
      DJIMotorSetPIDRef(inst, ref);
    } else {
      DJIMotorOuterLoop(inst, ANGLE_LOOP);
      float tgt = (m->mode == DART_MODE_TURNS) ? m->target * 360.0f : m->target;
      float ref = m->zero + SignOf(m) * tgt * m->ratio;  // 目标转子 total_angle
      DJIMotorSetPIDRef(inst, ref);
    }
  }
}
