# test_loopback.py — 仅验证 ESP32 的 UART 收发 (排除 STM32)
# 用法: 先用杜邦线把 ESP32 的 IO17 与 IO18 短接, 然后:
#   python -m mpremote connect COM6 run UserApp\robot\wifi_gm6020\esp32\test_loopback.py
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
time.sleep_ms(100)
u.read()
u.write(b"LOOP123\n")
time.sleep_ms(200)
d = u.read()
print("loopback RX:", d)
print("OK: ESP32 UART 正常" if (d and b"LOOP123" in d) else "FAIL: ESP32 UART 或 IO17/IO18 有问题")
