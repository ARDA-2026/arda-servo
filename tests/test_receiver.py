import json
import socket

from arda_servo.receiver import CoordReceiver


def _send(port: int, payload: dict) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(json.dumps(payload).encode("utf-8"), ("127.0.0.1", port))
    sock.close()


def test_recv_parses_valid_packet():
    receiver = CoordReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"x": 0.5, "y": 1.2, "z": 0.1, "fall": True, "confidence": 0.73, "ts": 123.0})
    coord = receiver.recv()

    assert coord is not None
    assert coord.x == 0.5
    assert coord.y == 1.2
    assert coord.z == 0.1
    assert coord.fall is True
    assert coord.confidence == 0.73

    receiver.close()


def test_recv_defaults_confidence_to_zero_when_missing():
    receiver = CoordReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"x": 0.5, "y": 1.2, "z": 0.1, "fall": True, "ts": 123.0})
    coord = receiver.recv()

    assert coord is not None
    assert coord.confidence == 0.0

    receiver.close()


def test_recv_times_out_when_no_data():
    receiver = CoordReceiver(host="127.0.0.1", port=0, timeout=0.1)
    assert receiver.recv() is None
    receiver.close()


def test_recv_ignores_malformed_packet():
    receiver = CoordReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"x": 0.5})  # y, z 누락
    assert receiver.recv() is None

    receiver.close()
