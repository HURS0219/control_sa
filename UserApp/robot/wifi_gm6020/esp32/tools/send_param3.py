# send_param3.py — 改 FF=123.45 并复位零点, 用于验证掉电保存
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
time.sleep_ms(100)
u.write("P,1,12345\n")   # FF=123.45
time.sleep_ms(200)
u.write("Z\n")           # 零点=当前ecd
time.sleep_ms(200)
print("sent P,1,12345 and Z")
