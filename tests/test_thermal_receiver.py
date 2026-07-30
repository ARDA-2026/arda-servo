import json
import socket

from arda_servo.thermal_receiver import ThermalPanReceiver


def _send(port: int, payload: dict) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(json.dumps(payload).encode("utf-8"), ("127.0.0.1", port))
    sock.close()


def test_recv_parses_valid_packet():
    receiver = ThermalPanReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"offset": 0.5, "ts": 123.0})
    pan = receiver.recv()

    assert pan is not None
    assert pan.offset == 0.5
    assert pan.ts == 123.0

    receiver.close()


def test_recv_clamps_offset_to_valid_range():
    receiver = ThermalPanReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"offset": 2.5, "ts": 0.0})
    pan = receiver.recv()

    assert pan is not None
    assert pan.offset == 1.0

    receiver.close()


def test_recv_times_out_when_no_data():
    receiver = ThermalPanReceiver(host="127.0.0.1", port=0, timeout=0.1)
    assert receiver.recv() is None
    receiver.close()


def test_recv_ignores_malformed_packet():
    receiver = ThermalPanReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"ts": 0.0})  # offset 누락
    assert receiver.recv() is None

    receiver.close()


def test_recv_parses_confirmed_with_vertical_offset():
    receiver = ThermalPanReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"confirmed": True, "vertical_offset": 0.7, "ts": 5.0})
    pan = receiver.recv()

    assert pan is not None
    assert pan.confirmed is True
    assert pan.vertical_offset == 0.7

    receiver.close()


def test_recv_confirmed_without_vertical_offset_is_none():
    receiver = ThermalPanReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"confirmed": True, "ts": 5.0})
    pan = receiver.recv()

    assert pan is not None
    assert pan.confirmed is True
    assert pan.vertical_offset is None

    receiver.close()


def test_recv_clamps_vertical_offset_to_valid_range():
    receiver = ThermalPanReceiver(host="127.0.0.1", port=0, timeout=1.0)
    port = receiver._sock.getsockname()[1]

    _send(port, {"confirmed": True, "vertical_offset": -3.0, "ts": 0.0})
    pan = receiver.recv()

    assert pan is not None
    assert pan.vertical_offset == -1.0

    receiver.close()
