from unittest.mock import patch

from arda_servo.controller import ServoController
from arda_servo.receiver import Coord
from arda_servo.servo import PanServo


class FakeReceiver:
    """CoordReceiver 대체용 — 큐에 넣은 Coord를 순서대로 반환."""

    def __init__(self, coords):
        self._coords = list(coords)

    def recv(self):
        return self._coords.pop(0) if self._coords else None

    def close(self):
        pass


def test_run_manual_updates_servo_angle_from_stdin():
    servo = PanServo(pin=33, simulate=True)
    controller = ServoController(servo, center_deg=90.0)

    with patch("builtins.input", side_effect=["0.0 1.0", "1.0 1.0", "q"]):
        controller.run_manual()

    assert servo.angle == 135.0  # atan2(1,1)=45° + center 90°


def test_run_manual_ignores_malformed_input():
    servo = PanServo(pin=33, simulate=True)
    controller = ServoController(servo, center_deg=90.0)

    with patch("builtins.input", side_effect=["oops", "0.5", "0.0 1.0", "q"]):
        controller.run_manual()

    assert servo.angle == 90.0


def test_run_forever_without_receiver_raises():
    servo = PanServo(pin=33, simulate=True)
    controller = ServoController(servo)

    try:
        controller.run_forever()
        assert False, "RuntimeError가 발생해야 한다"
    except RuntimeError:
        pass


def test_fall_coord_starts_dwell():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    controller = ServoController(servo, receiver, center_deg=90.0, dwell_seconds=3.0)

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    assert servo.angle == 135.0  # atan2(1,1)=45 + center 90
    assert controller._dwell_until == 103.0


def test_dwell_ignores_new_coords_until_expiry():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0),   # 100.0 — dwell 시작, until=103.0
        Coord(x=-1.0, y=1.0, z=0.0, fall=False, ts=0.0),  # 101.0 — dwell 중, 무시돼야 함
    ])
    controller = ServoController(servo, receiver, center_deg=90.0, dwell_seconds=3.0)

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()
    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()

    assert servo.angle == 135.0  # dwell 중이라 두 번째 좌표가 반영되지 않아야 함


def test_returns_home_after_dwell_expires():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0),    # 100.0 — dwell 시작, until=103.0
        Coord(x=-1.0, y=1.0, z=0.0, fall=False, ts=0.0),  # 104.0 — dwell 종료, 홈으로 복귀
    ])
    controller = ServoController(servo, receiver, center_deg=90.0, dwell_seconds=3.0)

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()
    with patch("arda_servo.controller.time.time", return_value=104.0):
        controller.step()

    # fall=False 좌표는 무시되고 홈 포지션(center_deg)으로 복귀해야 한다
    assert servo.angle == 90.0
    assert controller._dwell_until == 0.0


def test_non_fall_coords_are_ignored_when_idle():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=False, ts=0.0)])
    controller = ServoController(servo, receiver, center_deg=90.0)
    servo.set_angle(90.0)  # run_forever()가 하는 홈 이동을 흉내냄

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    # 낙하가 아닌 좌표는 무시되어 홈 포지션 그대로 유지돼야 한다
    assert servo.angle == 90.0
    assert controller._dwell_until == 0.0


def test_run_forever_moves_to_home_on_start():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([])  # recv()가 계속 None을 반환하도록

    class OneShotReceiver(FakeReceiver):
        def recv(self):
            raise KeyboardInterrupt  # 첫 step() 직후 루프를 바로 빠져나오기 위함

    controller = ServoController(servo, OneShotReceiver([]), center_deg=77.0)
    controller.run_forever()

    assert servo.angle == 77.0
