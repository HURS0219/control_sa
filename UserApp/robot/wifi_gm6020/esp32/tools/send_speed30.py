# send_speed30.py — 测试闭环: 目标 30 RPM 持续 10s
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
for i in range(50):
    u.write("C,1,30\n")
    time.sleep_ms(200)
print("sent C,1,30 x50")
