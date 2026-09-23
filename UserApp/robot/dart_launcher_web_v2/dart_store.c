/*
 * dart_store.c — 掉电保存: 四电机参数/零点/方向 + 舵机 + 拉簧圈数 + yaw 模式
 *
 * 仅在所有电机停止时写入, 避免擦写 Flash 卡住控制环。
 */
#include "dart_store.h"

#include <stdint.h>

#include "bsp_flash.h"
#include "dart_fsm.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "main.h"

#define CFG_ADDR  ADDR_FLASH_SECTOR_11
#define CFG_MAGIC 0xD1A7B002u  // v2

typedef struct {
  uint32_t magic;
  DartParam_s params[DART_MOTOR_COUNT];
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

static void Load(const DartCfg_s* cfg) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    g_dart_params[i] = cfg->params[i];
    g_dart_ctrl[i].zero = cfg->zero[i];
    g_dart_ctrl[i].zero_valid = 1;
    g_dart_ctrl[i].reverse = cfg->reverse[i] ? 1 : 0;
  }
  g_servo_std_deg = cfg->servo_std_deg;
  g_servo_prep_deg = cfg->servo_prep_deg;
  g_servo_offset_deg = cfg->servo_offset_deg;
  g_spring_turns = cfg->spring_turns;
  g_yaw_mode = cfg->yaw_mode;
}

void DartStoreInit(void) {
  DartCfg_s cfg;
  flash_read(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  if (cfg.magic == CFG_MAGIC) Load(&cfg);
}

static int MotorsStopped(void) {
  for (int i = 0; i < DART_MOTOR_COUNT; i++)
    if (g_dart_ctrl[i].mode != DART_MODE_STOP) return 0;
  if (g_task_step != DART_TASK_IDLE && g_task_step != DART_TASK_DONE) return 0;
  return 1;
}

void DartStoreTask(void) {
  if (!s_dirty) return;
  if (!MotorsStopped()) return;

  DartCfg_s cfg;
  cfg.magic = CFG_MAGIC;
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    cfg.params[i] = g_dart_params[i];
    cfg.zero[i] = g_dart_ctrl[i].zero;
    cfg.reverse[i] = g_dart_ctrl[i].reverse;
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
