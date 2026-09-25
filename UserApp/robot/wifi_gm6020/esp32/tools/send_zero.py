# send_zero.py — 测试复位(设零): 发 "Z"
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
time.sleep_ms(100)
u.write("Z\n")
time.sleep_ms(200)
print("sent Z")
