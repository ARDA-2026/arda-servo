import math
from unittest.mock import patch

from arda_servo.angle import elevation_range, pan_angle_to_xyz
from arda_servo.controller import ServoController
from arda_servo.receiver import Coord
from arda_servo.servo import PanServo
from arda_servo.site import local_to_latlon
from arda_servo.thermal_receiver import ThermalPan


class FakeReceiver:
    """CoordReceiver 대체용 — 큐에 넣은 Coord를 순서대로 반환."""

    def __init__(self, coords):
        self._coords = list(coords)

    def recv(self):
        return self._coords.pop(0) if self._coords else None

    def close(self):
        pass


class FakeThermalReceiver:
    """ThermalPanReceiver 대체용 — 큐에 넣은 ThermalPan을 순서대로 반환."""

    def __init__(self, pans):
        self._pans = list(pans)
        self.closed = False

    def recv(self):
        return self._pans.pop(0) if self._pans else None

    def close(self):
        self.closed = True


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


def test_dwell_expiry_holds_last_angle_without_returning_home():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0),    # 100.0 — dwell 시작, until=103.0
        Coord(x=-1.0, y=1.0, z=0.0, fall=False, ts=0.0),  # 104.0 — dwell 종료, 홈 복귀 없음
    ])
    controller = ServoController(servo, receiver, center_deg=90.0, dwell_seconds=3.0)

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()
    with patch("arda_servo.controller.time.time", return_value=104.0):
        controller.step()

    # fall=False 좌표는 무시되고, 서보는 홈(90.0)이 아니라 마지막 각도(135.0)에
    # 그대로 멈춰 있어야 한다 — 트리거가 있을 때만 움직인다.
    assert servo.angle == 135.0
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


def test_higher_confidence_coord_preempts_current_dwell():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, confidence=0.3, ts=0.0),  # 1차 낙하
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, confidence=0.6, ts=0.0),  # 더 확률 높은 후보
    ])
    controller = ServoController(servo, receiver, center_deg=90.0, dwell_seconds=10.0)

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=135.0 (atan2(1,1)=45+90), confidence=0.3

    assert servo.angle == 135.0
    assert controller._dwell_confidence == 0.3

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # confidence=0.6 > 0.3 → 즉시 선점 전환

    assert servo.angle == 45.0  # atan2(-1,1)=-45+90
    assert controller._dwell_confidence == 0.6
    assert controller._dwell_until == 111.0  # 101.0 + dwell_seconds(10.0)로 새로 시작됨


def test_lower_or_equal_confidence_coord_does_not_preempt():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, confidence=0.6, ts=0.0),
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, confidence=0.6, ts=0.0),  # 동일 확률 — 선점 아님
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, confidence=0.2, ts=0.0),  # 더 낮은 확률 — 선점 아님
    ])
    controller = ServoController(servo, receiver, center_deg=90.0, dwell_seconds=10.0)

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=135.0, confidence=0.6

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 동일 confidence → 무시
    with patch("arda_servo.controller.time.time", return_value=102.0):
        controller.step()  # 더 낮은 confidence → 무시

    assert servo.angle == 135.0  # 그대로 유지
    assert controller._dwell_confidence == 0.6


def test_confidence_preemption_still_works_before_thermal_engages():
    # 열화상이 아직 아무 보정도 안 보낸 상태(레이더 초기 조준만 된 상태)라면
    # confidence 기반 선점이 기존처럼 그대로 동작해야 한다.
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, confidence=0.3, ts=0.0),
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, confidence=0.9, ts=0.0),
    ])
    thermal = FakeThermalReceiver([None, None])  # 열화상 연동은 있지만 아직 아무것도 못 잡음
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0, thermal_receiver=thermal,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=135.0

    assert controller._thermal_engaged is False

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 열화상 미개입 상태 → confidence 선점 그대로 동작

    assert servo.angle == 45.0
    assert controller._dwell_confidence == 0.9


def test_thermal_engagement_blocks_confidence_preemption():
    # 열화상이 원하는 모양과 매칭된(matched=True) 보정을 한 번이라도 보내온
    # 뒤로는, 레이더가 더 높은 confidence의 새 좌표를 보내도 무시하고
    # 열화상이 서보 제어권을 계속 가져야 한다.
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, confidence=0.3, ts=0.0),
        None,
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, confidence=0.9, ts=0.0),
    ])
    thermal = FakeThermalReceiver([None, ThermalPan(offset=0.5, ts=0.0, matched=True)])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=135.0, 아직 열화상 미개입

    assert controller._thermal_engaged is False

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 매칭된 보정 수신 → angle=140.0, 이제부터 열화상이 제어권을 가짐

    assert servo.angle == 140.0
    assert controller._thermal_engaged is True

    with patch("arda_servo.controller.time.time", return_value=102.0):
        controller.step()  # 더 높은 확률(0.9>0.3)의 새 낙하가 와도 무시돼야 함

    assert servo.angle == 140.0  # 선점되지 않고 열화상이 추적하던 각도 그대로 유지
    assert controller._dwell_start_coord.confidence == 0.3  # 원래 좌표 그대로 유지됨


def test_thermal_pan_without_match_does_not_grant_control_and_allows_preemption():
    # 열이 감지됐지만 원하는 모양과 매칭되지 않은 보정(matched=False, 기본값)만
    # 온 경우 — 반사광/손처럼 사람이 아닌 열원에 제어권을 뺏기면 안 되므로
    # _thermal_engaged는 여전히 False여야 하고, 더 높은 confidence의 새 낙하가
    # 오면 정상적으로 선점돼야 한다.
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, confidence=0.3, ts=0.0),
        None,
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, confidence=0.9, ts=0.0),
    ])
    thermal = FakeThermalReceiver([None, ThermalPan(offset=0.5, ts=0.0)])  # matched=False(기본값)
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=135.0

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 매칭 안 된 보정 수신 → 각도는 따라가지만 제어권은 안 넘어감

    assert servo.angle == 140.0  # 팬 보정 자체는 matched 여부와 무관하게 적용됨
    assert controller._thermal_engaged is False

    with patch("arda_servo.controller.time.time", return_value=102.0):
        controller.step()  # 더 높은 확률(0.9>0.3)의 새 낙하는 선점돼야 함

    assert controller._dwell_start_coord.confidence == 0.9  # 새 후보로 정상 대체됨


def test_thermal_engaged_resets_for_next_dwell_cycle():
    # give_up으로 dwell이 끝나면 _thermal_engaged도 초기화돼, 다음 낙하
    # 트리거는 열화상 개입 여부와 무관하게 정상적으로 다시 시작해야 한다.
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, confidence=0.3, ts=0.0),
        None,
        None,
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, confidence=0.1, ts=0.0),
    ])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.5, ts=0.0, matched=True),
        ThermalPan(offset=0.0, ts=0.0, give_up=True),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=135.0
    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 매칭된 보정 → _thermal_engaged=True

    assert controller._thermal_engaged is True

    with patch("arda_servo.controller.time.time", return_value=102.0):
        controller.step()  # give_up → dwell 종료, _thermal_engaged 초기화

    assert controller._thermal_engaged is False

    with patch("arda_servo.controller.time.time", return_value=103.0):
        controller.step()  # 새 낙하(confidence=0.1이어도 idle 상태라 정상 수신)

    assert servo.angle == 45.0  # atan2(-1,1)=-45+90
    assert controller._thermal_engaged is False


def test_thermal_pan_applied_and_extends_dwell_during_tracking():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([None, ThermalPan(offset=0.5, ts=0.0)])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=3.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # 낙하 좌표 → angle=135.0, dwell_until=103.0

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 열화상 보정 offset=0.5 → angle += 5.0, dwell 연장

    assert servo.angle == 140.0
    assert controller._dwell_until == 104.0  # 101.0 + dwell_seconds(3.0)로 연장됨


def test_thermal_pan_with_vertical_offset_logs_position_every_frame(caplog):
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.5, ts=0.0, vertical_offset=0.2),
        ThermalPan(offset=-0.3, ts=0.0, vertical_offset=0.6),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=3.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
        install_height_m=1.3, camera_tilt_deg=50.0, vertical_fov_deg=35.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # 낙하 좌표 → dwell 시작

    with caplog.at_level("INFO"):
        with patch("arda_servo.controller.time.time", return_value=101.0):
            controller.step()  # 1차 보정 — vertical_offset=0.2로 거리 역산

        with patch("arda_servo.controller.time.time", return_value=102.0):
            controller.step()  # 2차 보정 — vertical_offset=0.6, 다른 거리로 다시 역산

    range_1 = elevation_range(0.2, 1.3, 50.0, 35.0)
    range_2 = elevation_range(0.6, 1.3, 50.0, 35.0)
    assert range_1 != range_2  # 프레임마다 다른 세로 위치로 거리가 매번 다시 계산돼야 함
    assert f"거리 {range_1:.2f}m" in caplog.text
    assert f"거리 {range_2:.2f}m" in caplog.text
    assert caplog.text.count("z=0 평면 기준") == 2  # 매 보정마다 역산 로그가 남아야 함


def test_thermal_pan_without_dwell_start_coord_skips_position_log(caplog):
    # _dwell_start_coord가 없는 경우(run_manual 등)에도 보정 자체는 정상 적용되고
    # 위치 로그만 생략돼야 한다.
    servo = PanServo(pin=33, simulate=True)
    controller = ServoController(
        servo, center_deg=90.0, dwell_seconds=3.0,
        install_height_m=1.3, camera_tilt_deg=50.0, vertical_fov_deg=35.0,
    )
    controller._dwell_until = 999999.0  # dwell 중인 것처럼 강제로 만듦 (_dwell_start_coord는 None으로 둠)
    thermal = FakeThermalReceiver([ThermalPan(offset=0.5, ts=0.0, vertical_offset=0.2)])
    controller._thermal_receiver = thermal
    controller._receiver = FakeReceiver([])

    with caplog.at_level("INFO"):
        with patch("arda_servo.controller.time.time", return_value=0.0):
            controller.step()

    assert "현재" not in caplog.text
    assert servo.angle == 94.0  # center_deg(90.0) + offset=0.5 * gain(기본 8.0) 만큼은 정상 반영됨


def test_thermal_pan_ignored_when_idle_at_home():
    servo = PanServo(pin=33, simulate=True)
    servo.set_angle(90.0)  # run_forever()가 하는 홈 이동을 흉내냄
    receiver = FakeReceiver([None])
    thermal = FakeThermalReceiver([ThermalPan(offset=1.0, ts=0.0)])
    controller = ServoController(servo, receiver, center_deg=90.0, thermal_receiver=thermal)

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    # dwell 중이 아니므로(홈 대기 상태) 열화상 보정은 무시돼야 한다
    assert servo.angle == 90.0


def test_thermal_pan_respects_invert():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=0.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([None, ThermalPan(offset=0.5, ts=0.0)])
    controller = ServoController(
        servo, receiver, center_deg=90.0, invert=True, dwell_seconds=3.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=90.0 (azimuth 0)

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # invert=True → offset 부호 반전 → angle -= 5.0

    assert servo.angle == 85.0


def test_thermal_give_up_holds_last_angle_immediately():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([None, ThermalPan(offset=0.0, ts=0.0, give_up=True)])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0, thermal_receiver=thermal,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # 낙하 좌표 → angle=135.0, dwell_until=110.0

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # give_up 수신 → dwell 만료를 기다리지 않고 즉시 종료(각도 유지)

    assert servo.angle == 135.0  # 홈으로 복귀하지 않고 마지막 각도에 그대로 멈춤
    assert controller._dwell_until == 0.0


def test_new_fall_coord_accepted_right_after_give_up():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([
        Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0),   # 100.0 — 1차 낙하, dwell 시작
        None,                                             # 101.0 — give_up과 같은 스텝, 좌표 없음
        Coord(x=-1.0, y=1.0, z=0.0, fall=True, ts=0.0),  # 102.0 — give_up 직후, 즉시 반응해야 함
    ])
    thermal = FakeThermalReceiver([None, ThermalPan(offset=0.0, ts=0.0, give_up=True), None])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0, thermal_receiver=thermal,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # angle=135.0 (atan2(1,1)=45+90), dwell 시작
    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # give_up → 즉시 종료(홈 복귀 없이 135.0에 그대로 멈춤)
    with patch("arda_servo.controller.time.time", return_value=102.0):
        controller.step()  # 새 낙하 좌표 즉시 반영돼야 함 (dwell 만료를 더 기다리지 않음)

    assert servo.angle == 45.0  # atan2(-1,1)=-45 + 90


def test_thermal_confirmed_holds_last_angle_and_logs_back_computed_coord(caplog):
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0), None])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.5, ts=0.0),               # 추적 중 각도가 더 움직임
        ThermalPan(offset=0.0, ts=0.0, confirmed=True),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # 낙하 좌표 → angle=135.0 (초기 조준), dwell 시작

    assert controller._dwell_start_coord == Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 열화상 보정 → angle=140.0으로 이동(추적)

    assert servo.angle == 140.0

    with caplog.at_level("WARNING"):
        with patch("arda_servo.controller.time.time", return_value=102.0):
            controller.step()  # confirmed 수신 → 즉시 종료 + 좌표 역산 로그(각도는 유지)

    assert servo.angle == 140.0  # 홈으로 복귀하지 않고 확정 당시 각도에 그대로 멈춤
    assert controller._dwell_until == 0.0
    assert controller._dwell_start_coord is None

    # range_m = hypot(1,1); 최종 각도 140°에서 역산한 좌표가 로그에 찍혀야 함
    range_m = math.hypot(1.0, 1.0)
    expected_x, expected_y = pan_angle_to_xyz(140.0, range_m, center_deg=90.0)
    assert f"x={expected_x:.2f}" in caplog.text
    assert f"y={expected_y:.2f}" in caplog.text
    assert "레이더 원좌표 x=1.00 y=1.00" in caplog.text


def test_thermal_confirmed_with_vertical_offset_uses_elevation_range(caplog):
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.0, ts=0.0, confirmed=True, vertical_offset=1.0),  # 화면 맨 아래 → 더 가까움
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal,
        install_height_m=1.3, camera_tilt_deg=50.0, vertical_fov_deg=35.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # 낙하 좌표 → angle=135.0, dwell 시작

    with caplog.at_level("WARNING"):
        with patch("arda_servo.controller.time.time", return_value=101.0):
            controller.step()  # confirmed(vertical_offset=1.0) → 카메라 기하로 거리 역산

    expected_range = elevation_range(1.0, 1.3, 50.0, 35.0)
    radar_range = math.hypot(1.0, 1.0)
    assert expected_range < radar_range  # 화면 아래쪽이므로 레이더 원거리보다 가까워야 함
    assert f"거리 {expected_range:.2f}m" in caplog.text
    assert "z=0 평면 기준" in caplog.text


def test_thermal_confirmed_without_vertical_offset_falls_back_to_radar_range(caplog):
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.0, ts=0.0, confirmed=True),  # vertical_offset 없음
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal,
        install_height_m=1.3, camera_tilt_deg=50.0, vertical_fov_deg=35.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    with caplog.at_level("WARNING"):
        with patch("arda_servo.controller.time.time", return_value=101.0):
            controller.step()

    radar_range = math.hypot(1.0, 1.0)
    assert f"거리 {radar_range:.2f}m" in caplog.text
    assert "레이더 원거리 유지" in caplog.text


def test_thermal_confirmed_with_site_config_logs_latlon_instead_of_local_xy(caplog):
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=2.0, y=3.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.0, ts=0.0, confirmed=True),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal,
        site_lat=37.5, site_lon=127.0, site_heading_deg=0.0,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    with caplog.at_level("WARNING"):
        with patch("arda_servo.controller.time.time", return_value=101.0):
            controller.step()

    orig_lat, orig_lon = local_to_latlon(2.0, 3.0, 37.5, 127.0, 0.0)
    assert f"lat={orig_lat:.6f} lon={orig_lon:.6f}" in caplog.text
    assert "x=2.00 y=3.00" not in caplog.text  # 로컬 좌표가 아니라 위경도로 남아야 함


def test_thermal_confirmed_without_site_config_falls_back_to_local_xy(caplog):
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=2.0, y=3.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.0, ts=0.0, confirmed=True),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0, thermal_receiver=thermal,
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    with caplog.at_level("WARNING"):
        with patch("arda_servo.controller.time.time", return_value=101.0):
            controller.step()

    assert "레이더 원좌표 x=2.00 y=3.00" in caplog.text
    assert "lat=" not in caplog.text


def test_thermal_confirmed_sends_corrected_coordinate_via_report_url():
    # report_url이 설정돼 있으면, 열화상 확정 시 레이더 최초 좌표(1,1)가
    # 아니라 열화상 추적으로 이동한 최종 각도(140°)에서 역산한 보정 좌표가
    # send_fall_report()로 전송돼야 한다.
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0), None])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.5, ts=0.0),               # 추적 중 각도가 더 움직임 (135° → 140°)
        ThermalPan(offset=0.0, ts=0.0, confirmed=True),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal, thermal_pan_gain_deg=10.0,
        site_lat=37.5, site_lon=127.0, site_heading_deg=0.0,
        report_url="http://example.invalid/report",
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()  # 낙하 좌표 수신 → angle=135.0

    with patch("arda_servo.controller.time.time", return_value=101.0):
        controller.step()  # 열화상 보정 → angle=140.0

    with patch("arda_servo.controller.send_fall_report") as mock_send:
        mock_send.return_value = True
        with patch("arda_servo.controller.time.time", return_value=102.0):
            controller.step()  # confirmed 수신

    assert mock_send.call_count == 1
    sent_url, sent_lat, sent_lon = mock_send.call_args[0]
    assert sent_url == "http://example.invalid/report"

    orig_lat, orig_lon = local_to_latlon(1.0, 1.0, 37.5, 127.0, 0.0)
    range_m = math.hypot(1.0, 1.0)
    final_x, final_y = pan_angle_to_xyz(140.0, range_m, center_deg=90.0)
    expected_lat, expected_lon = local_to_latlon(final_x, final_y, 37.5, 127.0, 0.0)

    assert (sent_lat, sent_lon) != (orig_lat, orig_lon)  # 원좌표 그대로면 안 됨
    assert sent_lat == expected_lat
    assert sent_lon == expected_lon


def test_thermal_confirmed_without_report_url_does_not_send():
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.0, ts=0.0, confirmed=True),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal,
        site_lat=37.5, site_lon=127.0,
        # report_url 미설정(기본 None) — 웹 전송 없이 로그만 남아야 함
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    with patch("arda_servo.controller.send_fall_report") as mock_send:
        with patch("arda_servo.controller.time.time", return_value=101.0):
            controller.step()

    mock_send.assert_not_called()


def test_thermal_confirmed_with_report_url_but_no_site_config_skips_send(caplog):
    servo = PanServo(pin=33, simulate=True)
    receiver = FakeReceiver([Coord(x=1.0, y=1.0, z=0.0, fall=True, ts=0.0)])
    thermal = FakeThermalReceiver([
        None,
        ThermalPan(offset=0.0, ts=0.0, confirmed=True),
    ])
    controller = ServoController(
        servo, receiver, center_deg=90.0, dwell_seconds=10.0,
        thermal_receiver=thermal,
        report_url="http://example.invalid/report",
        # site_lat/site_lon 미설정 — 위경도 변환이 안 되니 전송할 수 없다
    )

    with patch("arda_servo.controller.time.time", return_value=100.0):
        controller.step()

    with patch("arda_servo.controller.send_fall_report") as mock_send:
        with caplog.at_level("WARNING"):
            with patch("arda_servo.controller.time.time", return_value=101.0):
                controller.step()

    mock_send.assert_not_called()
    assert "위경도로 변환할 수 없음" in caplog.text


def test_run_forever_closes_thermal_receiver():
    servo = PanServo(pin=33, simulate=True)
    thermal = FakeThermalReceiver([])

    class OneShotReceiver(FakeReceiver):
        def recv(self):
            raise KeyboardInterrupt

    controller = ServoController(servo, OneShotReceiver([]), thermal_receiver=thermal)
    controller.run_forever()

    assert thermal.closed
