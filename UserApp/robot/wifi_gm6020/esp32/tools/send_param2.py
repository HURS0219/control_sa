# send_param2.py — 测试限幅随时可改: 设 g_max_value=8000
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
time.sleep_ms(100)
u.write("P,6,8000\n")
time.sleep_ms(200)
print("sent P,6,8000")
