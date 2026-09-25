# test_uart.py — ESP32 <-> STM32 串口链路自检
# 用法: python -m mpremote connect COM6 run UserApp\robot\wifi_gm6020\esp32\test_uart.py
#   1) 内部自环: 用同一引脚 tx=rx 验证 ESP32 UART 模块本身
#   2) 外部链路: ESP32 发 "PING\n", STM32 回 "PONG\n"
from machine import UART
import time, gc

# 释放 main.py 可能占用的 UART1
try:
    UART(1).deinit()
except Exception as e:
    print("deinit:", e)
gc.collect()
time.sleep_ms(100)

print("== 1) ESP32 internal loopback (tx=rx=17) ==")
u = UART(1, baudrate=115200, tx=17, rx=17)
time.sleep_ms(100)
u.read()
u.write(b"LOOP\n")
time.sleep_ms(200)
d = u.read()
print("  self RX:", d)
print("  self", "OK" if (d and b"LOOP" in d) else "FAIL(该固件可能不支持同脚自环)")
u.deinit()
gc.collect()

print("== 2) ESP32 <-> STM32 (tx=17 -> PG9, rx=18 <- PG14) ==")
uart = UART(1, baudrate=115200, tx=17, rx=18)
time.sleep_ms(200)
uart.read()
ok = 0
for i in range(10):
    uart.write("PING\n")
    time.sleep_ms(100)
    data = uart.read()
    if data:
        print("  RX:", data)
        if b"PONG" in data:
            ok += 1
    time.sleep_ms(200)
print("  PONG received: %d / 10" % ok)
print("  " + ("OK: 双向串口通信正常" if ok else "FAIL: 无响应, 检查 GPIO17->PG9 / GPIO18<-PG14 / 共地, 以及 STM32 是否上电"))
