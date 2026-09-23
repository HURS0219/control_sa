/*
 * dart_store.c — 掉电保存: 四电机 PID(11项)/零点/方向 + 舵机 + 拉簧圈数 + yaw 模式
 *
 * 仅在所有电机停止时写入。
 */
#include "dart_store.h"

#include <stdint.h>

#include "bsp_flash.h"
#include "dart_fsm.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "main.h"

#define CFG_ADDR  ADDR_FLASH_SECTOR_11
#define CFG_MAGIC 0xD1A7B006u  // v3 (放开角度限幅默认值)

typedef struct {
  uint32_t magic;
  float params[DART_MOTOR_COUNT][11];
  float zero[DART_MOTOR_COUNT];
  uint8_t reverse[DART_MOTOR_COUNT];
  uint8_t pad[4];
  float servo_std_deg;
  float servo_prep_deg;
  float servo_offset_deg;
  float spring_turns;
  int32_t yaw_mode;
} DartCfg_s;

static volatile uint8_t s_dirty = 0;
static volatile uint8_t s_saved_flag = 0;

void DartStoreMarkDirty(void) { s_dirty = 1; }
int DartStoreTakeSaved(void) {
  int v = s_saved_flag;
  s_saved_flag = 0;
  return v;
}

/* 从 PIDInstance 读出 11 项参数 */
static void ReadParams(int slot, float* out) {
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

void DartStoreInit(void) {
  DartCfg_s cfg;
  flash_read(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  if (cfg.magic != CFG_MAGIC) return;

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    for (int id = 1; id <= 11; id++) DartMotorLoadParamRaw(i, id, cfg.params[i][id - 1]);
    DartMotorLoadZero(i, cfg.zero[i], 1);
    DartMotorLoadDir(i, cfg.reverse[i]);
  }
  g_servo_std_deg = cfg.servo_std_deg;
  g_servo_prep_deg = cfg.servo_prep_deg;
  g_servo_offset_deg = cfg.servo_offset_deg;
  g_spring_turns = cfg.spring_turns;
  g_yaw_mode = cfg.yaw_mode;
}

static int MotorsStopped(void) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++)
    if (g_dart_motors[i].mode != DART_MODE_STOP) return 0;
  if (g_task_step != DART_TASK_IDLE && g_task_step != DART_TASK_DONE) return 0;
  return 1;
}

void DartStoreTask(void) {
  if (!s_dirty) return;
  if (!MotorsStopped()) return;

  DartCfg_s cfg;
  cfg.magic = CFG_MAGIC;
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    ReadParams(i, cfg.params[i]);
    cfg.zero[i] = g_dart_motors[i].zero;
    cfg.reverse[i] =
        (g_dart_motors[i].inst->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? 1
                                                                                              : 0;
  }
  for (int i = 0; i < 4; i++) cfg.pad[i] = 0;
  cfg.servo_std_deg = g_servo_std_deg;
  cfg.servo_prep_deg = g_servo_prep_deg;
  cfg.servo_offset_deg = g_servo_offset_deg;
  cfg.spring_turns = g_spring_turns;
  cfg.yaw_mode = g_yaw_mode;

  flash_erase_address(CFG_ADDR, 1);
  flash_write_single_address(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  s_dirty = 0;
  s_saved_flag = 1;
}
