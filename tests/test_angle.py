from arda_servo.angle import xyz_to_pan_angle


def test_center_when_directly_ahead():
    assert xyz_to_pan_angle(x=0.0, y=1.0) == 90.0


def test_right_side_increases_angle():
    angle = xyz_to_pan_angle(x=1.0, y=1.0)
    assert angle > 90.0


def test_left_side_decreases_angle():
    angle = xyz_to_pan_angle(x=-1.0, y=1.0)
    assert angle < 90.0


def test_invert_flips_direction():
    normal = xyz_to_pan_angle(x=1.0, y=1.0)
    inverted = xyz_to_pan_angle(x=1.0, y=1.0, invert=True)
    assert normal > 90.0
    assert inverted < 90.0


def test_clamped_to_max_deg():
    angle = xyz_to_pan_angle(x=10.0, y=0.01, max_deg=120.0)
    assert angle == 120.0


def test_clamped_to_min_deg():
    angle = xyz_to_pan_angle(x=-10.0, y=0.01, min_deg=60.0)
    assert angle == 60.0


def test_nonpositive_y_does_not_raise():
    angle = xyz_to_pan_angle(x=1.0, y=0.0)
    assert 0.0 <= angle <= 180.0
