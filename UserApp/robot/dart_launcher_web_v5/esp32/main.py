# main.py — 制导飞镖发射架 · 网页控制台 v2 (ESP32-S3 / MicroPython)
# 组装: 总线驱动 Bus + 电机/舵机/任务对象 + HTTP 服务
#
# 依赖同目录: proto.py motor.py servo.py task.py web.py 以及 www/ 静态文件。

import network
import time

from proto import Bus
from motor import M3508, M2006
from servo import PTK7350
from task import Task
from web import WebServer

AP_SSID = "DART_CTRL"
AP_PASS = "12345678"

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=AP_SSID, password=AP_PASS, max_clients=4)
while not ap.active():
    time.sleep_ms(50)
print("AP ready:", ap.ifconfig()[0])

# 总线
bus = Bus()

# 设备对象 (slot 顺序与 C 端角色一致)
motors = [
    M3508(bus, 0, "拉簧A", 2),
    M3508(bus, 1, "拉簧B", 3),
    M3508(bus, 2, "扳机", 4),
    M2006(bus, 3, "Yaw", 1),
]
servo = PTK7350(bus, "trigger")
task = Task(bus)

# 网页服务
web = WebServer(bus, motors, servo, task)
print("HTTP server listening on :80")

last_hb = 0
while True:
    bus.poll()
    web.poll_http()
    now = time.ticks_ms()
    if time.ticks_diff(now, last_hb) > 500:
        last_hb = now
        bus.send("H")
    time.sleep_ms(5)
