from unittest.mock import patch

from arda_servo.controller import ServoController
from arda_servo.servo import PanServo


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
