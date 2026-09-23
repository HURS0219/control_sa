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
#define CFG_MAGIC 0xD1A7B009u  // v4 (临界区保存 + 拉簧同向 + 更软默认)

typedef struct {
  uint32_t magic;
  float params[MOTOR_COUNT][11];
  float zero[MOTOR_COUNT];
  uint8_t reverse[MOTOR_COUNT];
  uint8_t pad[4];
  float servo_std_deg;
  float servo_prep_deg;
  float servo_offset_deg;
  float spring_turns;
  int32_t yaw_mode;
} DartCfg_s;

static volatile uint8_t s_dirty = 0;
static volatile uint8_t s_force = 0;
static volatile uint32_t s_dirty_tick = 0;
static volatile uint8_t s_saved_flag = 0;

void DartStoreMarkDirty(void) {
  s_dirty = 1;
  s_dirty_tick = HAL_GetTick();
}
void DartStoreMarkDirtyNow(void) {
  s_dirty = 1;
  s_force = 1;
  s_dirty_tick = HAL_GetTick();
}
int DartStoreTakeSaved(void) {
  int v = s_saved_flag;
  s_saved_flag = 0;
  return v;
}

/* 从 PIDInstance 读出 11 项参数 */
static void ReadParams(int slot, float* out) {
  PIDInstance* sp = &Axis[slot].inst->motor_controller.speed_PID;
  PIDInstance* ap = &Axis[slot].inst->motor_controller.angle_PID;
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
  out[10] = Axis[slot].ratio;
}

void DartStoreInit(void) {
  DartCfg_s cfg;
  flash_read(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  if (cfg.magic != CFG_MAGIC) return;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    for (int id = 1; id <= 11; id++) MotorLoadParamRaw(i, id, cfg.params[i][id - 1]);
    MotorLoadZero(i, cfg.zero[i], 1);
    MotorLoadDir(i, cfg.reverse[i]);
  }
  g_servo_std_deg = cfg.servo_std_deg;
  g_servo_prep_deg = cfg.servo_prep_deg;
  g_servo_offset_deg = cfg.servo_offset_deg;
  g_spring_turns = cfg.spring_turns;
  g_yaw_mode = cfg.yaw_mode;
}

static int MotorsStopped(void) {
  for (int i = 0; i < MOTOR_COUNT; i++)
    if (Axis[i].mode != MODE_STOP) return 0;
  if (g_task_step != DART_TASK_IDLE && g_task_step != DART_TASK_DONE) return 0;
  return 1;
}

void DartStoreTask(void) {
  if (!s_dirty) return;
  if (!MotorsStopped()) return;
  /* 去抖: 设定稳定 600ms 后再写; 显式“立即保存”跳过去抖 */
  if (!s_force && HAL_GetTick() - s_dirty_tick < 600u) return;
  s_force = 0;
  s_dirty_tick = HAL_GetTick();  // 若写入过程中又有改动, 下一轮再存

  DartCfg_s cfg;
  cfg.magic = CFG_MAGIC;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    ReadParams(i, cfg.params[i]);
    cfg.zero[i] = Axis[i].zero;
    cfg.reverse[i] =
        (Axis[i].inst->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ? 1
                                                                                              : 0;
  }
  for (int i = 0; i < 4; i++) cfg.pad[i] = 0;
  cfg.servo_std_deg = g_servo_std_deg;
  cfg.servo_prep_deg = g_servo_prep_deg;
  cfg.servo_offset_deg = g_servo_offset_deg;
  cfg.spring_turns = g_spring_turns;
  cfg.yaw_mode = g_yaw_mode;

  __disable_irq();  // 擦写期间关中断, 防止被中断打断导致擦除失败
  flash_erase_address(CFG_ADDR, 1);
  flash_write_single_address(CFG_ADDR, (uint32_t*)&cfg, sizeof(cfg) / 4);
  __enable_irq();
  s_dirty = 0;
  s_saved_flag = 1;
}
