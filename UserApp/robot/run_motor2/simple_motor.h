/*
 * simple_motor.h — 最简通用 DJI 电机直接 CAN 驱动 (GM6020 / M3508 / M2006)
 *
 * 不依赖 dji_motor / 电机任务, 直接 HAL 发控制帧、收反馈帧。
 *
 * CAN 差异:
 *   电机     控制帧(ID1-4 / ID5-8)     数值范围     反馈帧
 *   GM6020   0x1FF / 0x2FF             ±30000       0x204+ID
 *   M3508    0x200 / 0x1FF             ±16384       0x200+ID
 *   M2006    0x200 / 0x1FF             ±10000       0x200+ID
 *   控制帧 8 字节 = 4 电机 x 2 字节(大端 int16), 按 (ID-1)%4 占槽
 *   反馈帧 8 字节: [0:1]编码器 [2:3]转速RPM [4:5]电流 [6]温度
 */
#ifndef WIFI_GM6020_SIMPLE_MOTOR_H
#define WIFI_GM6020_SIMPLE_MOTOR_H

#include <stdint.h>

typedef enum {
  SIMPLE_MOTOR_GM6020 = 0,
  SIMPLE_MOTOR_M3508,
  SIMPLE_MOTOR_M2006,
} SimpleMotorType_e;

typedef struct {
  SimpleMotorType_e type;
  uint8_t id;  // 电调拨码 ID
} SimpleMotorConfig_s;

typedef struct {
  uint16_t ecd;
  int16_t rpm;
  int16_t current;
  uint8_t temp;
  uint32_t fb_count;
} SimpleMotorInstance;

extern SimpleMotorInstance g_simple_motor;

void SimpleMotorInit(const SimpleMotorConfig_s* cfg);
void SimpleMotorSet(int16_t value);   // 自动按电机类型限幅
void SimpleMotorReadFeedback(void);

#endif  // WIFI_GM6020_SIMPLE_MOTOR_H
