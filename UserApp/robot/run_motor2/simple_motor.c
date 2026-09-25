/*
 * simple_motor.c — 最简通用 DJI 电机直接 CAN 实现 (GM6020 / M3508 / M2006)
 */
#include "simple_motor.h"

#include "can.h"  // hcan1
#include "main.h"

SimpleMotorInstance g_simple_motor;
static SimpleMotorConfig_s s_cfg;

/* 控制帧 ID */
static uint16_t CtrlFrameId(void) {
  switch (s_cfg.type) {
    case SIMPLE_MOTOR_GM6020:
      return (s_cfg.id <= 4) ? 0x1FF : 0x2FF;
    case SIMPLE_MOTOR_M3508:
    case SIMPLE_MOTOR_M2006:
    default:
      return (s_cfg.id <= 4) ? 0x200 : 0x1FF;
  }
}

/* 反馈帧 ID */
static uint16_t FeedbackId(void) {
  if (s_cfg.type == SIMPLE_MOTOR_GM6020) return (uint16_t)(0x204 + s_cfg.id);
  return (uint16_t)(0x200 + s_cfg.id);
}

/* 按电机类型限幅 */
static int16_t ClampValue(int32_t v) {
  int32_t lim;
  switch (s_cfg.type) {
    case SIMPLE_MOTOR_GM6020: lim = 30000; break;
    case SIMPLE_MOTOR_M3508: lim = 16384; break;
    case SIMPLE_MOTOR_M2006: lim = 10000; break;
    default: lim = 30000; break;
  }
  if (v > lim) v = lim;
  else if (v < -lim) v = -lim;
  return (int16_t)v;
}

void SimpleMotorInit(const SimpleMotorConfig_s* cfg) {
  s_cfg = *cfg;

  /* 接收滤波器: 只收本电机反馈帧 */
  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterIdHigh = (uint16_t)(FeedbackId() << 5);
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &filter);

  HAL_CAN_Start(&hcan1);  // main.c 的 MX_CAN1_Init 只做了 HAL_CAN_Init
}

void SimpleMotorSet(int16_t value) {
  value = ClampValue(value);

  uint8_t data[8] = {0};
  uint8_t slot = (uint8_t)((s_cfg.id - 1) % 4);
  data[slot * 2] = (uint8_t)(value >> 8);
  data[slot * 2 + 1] = (uint8_t)(value & 0xFF);

  CAN_TxHeaderTypeDef tx = {0};
  tx.StdId = CtrlFrameId();
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;
  tx.TransmitGlobalTime = DISABLE;

  uint32_t t0 = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    if (HAL_GetTick() - t0 > 5) return;
  }

  uint32_t mailbox = 0;
  HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

void SimpleMotorReadFeedback(void) {
  CAN_RxHeaderTypeDef rx;
  uint8_t buf[8];
  uint16_t fb_id = FeedbackId();

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, buf) != HAL_OK) break;
    if (rx.StdId == fb_id) {
      g_simple_motor.ecd = (uint16_t)((buf[0] << 8) | buf[1]);
      g_simple_motor.rpm = (int16_t)((buf[2] << 8) | buf[3]);
      g_simple_motor.current = (int16_t)((buf[4] << 8) | buf[5]);
      g_simple_motor.temp = buf[6];
      g_simple_motor.fb_count++;
    }
  }
}
