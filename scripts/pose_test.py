import json
import socket
import time


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
angles = [90.0] * 32
angles[14] = 100.0
angles[17] = 40.0
angles[19] = 40.0

payload = {
    "frameId": 2,
    "timestampNs": time.time_ns(),
    "timestampRfc3339": "",
    "source": "udp_pose_test",
    "angles": angles,
}

sock.sendto(json.dumps(payload).encode("utf-8"), ("127.0.0.1", 6000))
print("sent udp pose frame to 127.0.0.1:6000")
