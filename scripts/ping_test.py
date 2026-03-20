import time
import zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 3000)
sock.setsockopt(zmq.SNDTIMEO, 3000)
sock.connect("tcp://192.168.137.93:5555")

payload = {
    "command": "ping",
    "client_time_ns": time.time_ns(),
    "source": "pc_ping_test",
}

sock.send_json(payload)
print(sock.recv_json())
