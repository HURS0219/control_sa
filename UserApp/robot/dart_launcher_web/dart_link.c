/*
 * dart_link.c — ESP32 <-> C 板 串口协议
 *
 * ESP32 -> STM32 (ASCII, '\n'):
 *   PING                      链路自检, 回 PONG
 *   S                         扫描 CAN 反馈 ID, 回 S,<n>,<id>...
 *   U,<slot>                  选择调参槽位 (0..3)
 *   P,<id>,<value>            设定所选电机参数 (浮点 x100, 见 dart_motor.c)
 *   R,<slot>                  恢复某电机默认参数
 *   Z / Z,<slot>              设零 (当前位置=角度 0)
 *   M,<slot>,<mode>,<value>   电机控制 mode 0停/1速度(rpm x10)/2角度(deg x10)
 *   V,<a>,<b>                 舵机: 0设标准位us 1设预备位us 2去标准位 3去预备位 4直接us
 *   G,<cmd>                   状态机: 0拉簧标准 1拉簧预备 2舵机标准 3舵机预备
 *                                      10自动开始 11自动停止 12急停 13解除急停
 *   W,<turns_x100>            拉簧预备位圈数
 *   Y,<mode>                  yaw 模式 0手动 1自瞄 2制导(0度)
 *   C,<x>,<center>            注入视觉绿光坐标 (联调用)
 *
 * STM32 -> ESP32:
 *   PONG
 *   S,<n>,<id>...
 *   F,<n>,<blk>*n,<sel>,<11参数>,<舵机4>,<拉簧/状态4>,<视觉4>   (100ms)
 *     blk = <slot>,<type>,<id>,<online>,<rpm>,<pos_x10>,<temp>,<cur>
 */
#include "dart_link.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_usart.h"
#include "dart_fsm.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "dart_store.h"
#include "dart_vision.h"
#include "main.h"
#include "robot_config.h"
#include "simple_motor.h"

static USARTInstance* s_usart = NULL;
static uint32_t s_last_cmd_tick = 0;

/* 扫描状态 */
static uint8_t s_scan_active = 0;
static uint32_t s_scan_t0 = 0;

/* 把逗号分隔的整数解析到 out, 返回个数 */
static int SplitInts(const char* s, long* out, int max) {
  int n = 0;
  const char* p = s;
  while (*p && n < max) {
    char* end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    out[n++] = v;
    p = end;
    if (*p == ',') p++;
    else break;
  }
  return n;
}

/* ================= 命令处理 ================= */
static void DartDecode(void) {
  char* buf = (char*)s_usart->recv_buff;
  s_last_cmd_tick = HAL_GetTick();

  if (buf[0] == 'P' && buf[1] == 'I' && buf[2] == 'N' && buf[3] == 'G') {
    DartLinkSendBlocking("PONG\n");
    return;
  }

  char cmd = buf[0];
  long a[4] = {0};
  int n = (buf[1] == ',') ? SplitInts(buf + 2, a, 4) : 0;

  switch (cmd) {
    case 'S':
      if (!g_dart_scanning) s_scan_active = 1;
      break;

    case 'Z':
      DartMotorZero(n > 0 ? (int)a[0] : -1);
      DartStoreMarkDirty();
      break;

    case 'U':
      if (n >= 1) DartMotorSelect((int)a[0]);
      break;

    case 'P':
      if (n >= 2) {
        DartMotorSetParam((int)a[0], (int)a[1]);
        DartStoreMarkDirty();
      }
      break;

    case 'R':
      if (n >= 1) {
        DartMotorResetParams((int)a[0]);
        DartStoreMarkDirty();
      }
      break;

    case 'M':
      if (n >= 3) {
        float value = (a[1] == 1 || a[1] == 2) ? (float)a[2] / 10.0f : 0.0f;
        DartMotorSetCtrl((int)a[0], (int)a[1], value);
      }
      break;

    case 'V':
      if (n >= 1) {
        switch (a[0]) {
          case 0: DartServoSetPos(DART_SERVO_STD, (float)a[1]); DartStoreMarkDirty(); break;
          case 1: DartServoSetPos(DART_SERVO_PREP, (float)a[1]); DartStoreMarkDirty(); break;
          case 2: DartServoGo(DART_SERVO_STD); break;
          case 3: DartServoGo(DART_SERVO_PREP); break;
          case 4: DartServoSetUs((float)a[1]); break;
          default: break;
        }
      }
      break;

    case 'G':
      if (n >= 1) {
        switch (a[0]) {
          case 0: DartFsmSpringGo(0); break;
          case 1: DartFsmSpringGo(1); break;
          case 2: DartFsmServoGo(DART_SERVO_STD); break;
          case 3: DartFsmServoGo(DART_SERVO_PREP); break;
          case 10: DartFsmAutoStart(); break;
          case 11: DartFsmAutoStop(); break;
          case 12: DartFsmSetEstop(1); break;
          case 13: DartFsmSetEstop(0); break;
          default: break;
        }
      }
      break;

    case 'W':
      if (n >= 1) {
        g_spring_turns = (float)a[0] / 100.0f;
        DartStoreMarkDirty();
      }
      break;

    case 'Y':
      if (n >= 1) {
        g_yaw_mode = (int)a[0];
        DartStoreMarkDirty();
      }
      break;

    case 'C':
      if (n >= 1) DartVisionSet((int)a[0], n >= 2 ? (int)a[1] : 0);
      break;

    default:
      break;
  }
}

/* ================= 扫描 ================= */
static void ScanTask(void) {
  if (!s_scan_active) return;

  if (s_scan_t0 == 0) {
    s_scan_t0 = HAL_GetTick();
    g_dart_scanning = 1;
    SimpleMotorScanBegin();
    return;
  }

  SimpleMotorScanTask();

  if (HAL_GetTick() - s_scan_t0 >= 1000u) {
    uint16_t ids[8];
    uint8_t n = SimpleMotorScanResult(ids, 8);
    static char buf[96];
    int len = snprintf(buf, sizeof(buf), "S,%d", n);
    for (uint8_t i = 0; i < n && len < (int)sizeof(buf) - 8; i++)
      len += snprintf(buf + len, sizeof(buf) - len, ",%d", ids[i]);
    buf[len++] = '\n';
    buf[len] = 0;
    DartLinkSendBlocking(buf);
    SimpleMotorReconfigFilters();
    g_dart_scanning = 0;
    s_scan_active = 0;
    s_scan_t0 = 0;
  }
}

/* ================= 遥测 ================= */
/* 帧: F,<n>,<每个电机8项>*n,<sel>,<每个电机11项参数>*n,<舵机4>,<拉簧/状态4>,<视觉4> */
static void SendTelemetry(void) {
  static char buf[768];
  int len = snprintf(buf, sizeof(buf), "F,%d", DART_MOTOR_COUNT);

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    SimpleMotor_t* m = &g_motors[i];
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d,%d,%d,%d", i, (int)m->type,
                    (int)m->id, (int)g_dart_ctrl[i].online, (int)m->rpm,
                    (int)(m->total_angle * 10.0f), (int)m->temp, (int)m->current);
  }

  len += snprintf(buf + len, sizeof(buf) - len, ",%d", g_dart_sel);
  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    DartParam_s* p = &g_dart_params[i];
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    (int)(p->speed_ff * 100), (int)(p->speed_kp * 100), (int)(p->speed_ki * 100),
                    (int)(p->speed_i_limit * 100), (int)(p->angle_kp * 100), (int)(p->angle_kd * 100),
                    (int)(p->angle_slew * 100), (int)(p->gear_ratio * 100),
                    (int)(p->angle_deadband * 100), (int)(p->angle_speed_max * 100), (int)p->max_value);
  }

  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", (int)g_servo_cur_us, g_servo_state,
                  (int)g_servo_std_us, (int)g_servo_prep_us);
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", (int)(g_spring_turns * 100),
                  g_auto_step, g_yaw_mode, g_estop);
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", g_vis_x, g_vis_ok, g_vis_center,
                  g_vis_err);

  buf[len++] = '\n';
  buf[len] = 0;
  DartLinkSend(buf);
}

void DartLinkInit(void) {
  USART_Init_Config_s cfg = {
      .recv_buff_size = DART_RECV_SIZE,
      .usart_handle = DART_UART_HANDLE,
      .module_callback = DartDecode,
  };
  s_usart = USARTRegister(&cfg);
}

void DartLinkTask(void) {
  ScanTask();

  /* 网页断联保护: 超时无任何指令则安全停车 */
  static uint8_t timed_out = 0;
  if (s_last_cmd_tick != 0 && HAL_GetTick() - s_last_cmd_tick > DART_CMD_TIMEOUT_MS) {
    if (!timed_out) {
      timed_out = 1;
      DartMotorStopAll();
      DartFsmAutoStop();
    }
  } else {
    timed_out = 0;
  }

  static uint32_t last_fb = 0;
  if (HAL_GetTick() - last_fb >= DART_FB_PERIOD_MS) {
    last_fb = HAL_GetTick();
    SendTelemetry();
  }
}

void DartLinkSend(const char* s) {
  if (s_usart == NULL || s == NULL) return;
  USARTSend(s_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}

void DartLinkSendBlocking(const char* s) {
  if (s_usart == NULL || s == NULL) return;
  uint32_t t0 = HAL_GetTick();
  while (s_usart->usart_handle->gState != HAL_UART_STATE_READY) {
    if (HAL_GetTick() - t0 > 50) break;
  }
  USARTSend(s_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_BLOCKING);
}
