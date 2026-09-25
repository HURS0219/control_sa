# send_ping.py — 仅从 ESP32 发送 PING, 供 JLink 读取 STM32 接收计数
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
time.sleep_ms(100)
for i in range(30):
    u.write("PING\n")
    time.sleep_ms(100)
print("sent 30 PING")
