# send_angle.py — 测试角度模式: 先锁零点, 再转到 +90.0 度 (900 = 90.0*10)
from machine import UART
import time, gc

try:
    UART(1).deinit()
except Exception:
    pass
gc.collect()

u = UART(1, baudrate=115200, tx=17, rx=18)
# 进入角度模式并把当前位置锁为零点
for i in range(5):
    u.write("C,2,0\n")
    time.sleep_ms(200)
# 转到 +90.0 度并保持
for i in range(40):
    u.write("C,2,900\n")
    time.sleep_ms(200)
print("done angle test")
