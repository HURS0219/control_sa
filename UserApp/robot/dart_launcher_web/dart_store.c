/*
 * dart_store.c — 掉电保存: 四电机参数/零点 + 舵机位 + 拉簧圈数 + yaw 模式
 *
 * 只在非运行 (电机全部 STOP 且非自动流程) 时写入, 避免边跑边擦写 Flash。
 */
#include "dart_store.h"

#include <stdint.h>

#include "bsp_flash.h"
#include "dart_fsm.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "main.h"

#define CFG_ADDR  ADDR_FLASH_SECTOR_11
#define CFG_MAGIC 0xD1A7B001u  // 结构变更后递增

typedef struct {
  uint32_t magic;
  DartParam_s params[DART_MOTOR_COUNT];
  float zero[DART_MOTOR_COUNT];
  float servo_std_us;
  float servo_prep_us;
  float spring_turns;
  int32_t yaw_mode;
} DartCfg_s;

static volatile uint8_t s_dirty = 0;

void DartStoreMarkDirty(void) { s_dirty = 1; }

void DartStoreInit(void) {
  DartCfg_s cfg;
  flash_read(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  if (cfg.magic != CFG_MAGIC) return;

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    g_dart_params[i] = cfg.params[i];
    g_dart_ctrl[i].zero = cfg.zero[i];
    g_dart_ctrl[i].zero_valid = 1;
  }
  g_servo_std_us = cfg.servo_std_us;
  g_servo_prep_us = cfg.servo_prep_us;
  g_spring_turns = cfg.spring_turns;
  g_yaw_mode = cfg.yaw_mode;

  DartServoGo(g_servo_state);  // 用新参数刷新当前舵机位
}

void DartStoreTask(void) {
  if (!s_dirty) return;

  /* 仅在所有电机停止、非自动流程时写入 */
  for (int i = 0; i < DART_MOTOR_COUNT; i++)
    if (g_dart_ctrl[i].mode != DART_CTRL_STOP) return;
  if (g_auto_step != DART_AUTO_IDLE && g_auto_step != DART_AUTO_DONE) return;

  DartCfg_s cfg;
  cfg.magic = CFG_MAGIC;
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    cfg.params[i] = g_dart_params[i];
    cfg.zero[i] = g_dart_ctrl[i].zero;
  }
  cfg.servo_std_us = g_servo_std_us;
  cfg.servo_prep_us = g_servo_prep_us;
  cfg.spring_turns = g_spring_turns;
  cfg.yaw_mode = g_yaw_mode;

  flash_erase_address(CFG_ADDR, 1);
  flash_write_single_address(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  s_dirty = 0;
}
