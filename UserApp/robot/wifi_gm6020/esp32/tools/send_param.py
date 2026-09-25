# send_param.py — 测试参数设置: FF=95.00, Kp=40.00
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
time.sleep_ms(100)
u.write("P,1,9500\n")   # id1 SPEED_FF = 95.00
time.sleep_ms(200)
u.write("P,2,4000\n")   # id2 SPEED_KP = 40.00
time.sleep_ms(200)
print("sent P,1,9500 and P,2,4000")
