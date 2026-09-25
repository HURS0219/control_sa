# send_cmd.py — 直接发速度指令 C,1,100 持续 10s (绕过网页, 配合 JLink 观察)
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
for i in range(50):
    u.write("C,1,100\n")
    time.sleep_ms(200)
print("sent C,1,100 x50")
