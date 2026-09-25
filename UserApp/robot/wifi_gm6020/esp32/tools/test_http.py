# test_http.py — 在 ESP32 本机自测 HTTP 服务端收发逻辑 (不依赖手机)
# 用法: python -m mpremote connect COM6 run UserApp\robot\wifi_gm6020\esp32\test_http.py
import socket
import time

BODY = b"x" * 2500  # 模拟网页大小


def send_all(sock, data):
    mv = memoryview(data)
    while len(mv):
        n = sock.send(mv)
        if n is None:
            break
        mv = mv[n:]


srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(socket.getaddrinfo("0.0.0.0", 8080)[0][-1])
srv.listen(1)
srv.settimeout(3)

c = socket.socket()
c.settimeout(3)
try:
    c.connect(("192.168.4.1", 8080))
    c.send(b"GET / HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n")
    cl, _ = srv.accept()
    req = cl.recv(512)
    hdr = ("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nConnection: close\r\n\r\n" % len(BODY)).encode()
    send_all(cl, hdr)
    send_all(cl, BODY)
    cl.close()

    data = b""
    while True:
        d = c.recv(1024)
        if not d:
            break
        data += d
    print("client got %d bytes" % len(data))
    print("header:", data.split(b"\r\n\r\n", 1)[0][:80])
    print("OK: HTTP 服务端逻辑正常" if data.endswith(b"x") and len(data) > 2500 else "FAIL")
except Exception as e:
    print("ERR:", e)
finally:
    c.close()
    srv.close()
