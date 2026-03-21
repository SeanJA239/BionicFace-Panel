import json
import socket
import time


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
payload = {
    "frameId": 1,
    "timestampNs": time.time_ns(),
    "timestampRfc3339": "",
    "source": "udp_ping_test",
    "angles": [90.0] * 32,
}

sock.sendto(json.dumps(payload).encode("utf-8"), ("127.0.0.1", 6000))
print("sent udp frame to 127.0.0.1:6000")
