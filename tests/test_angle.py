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


def test_offset_zero_matches_no_offset():
    with_zero_offset = xyz_to_pan_angle(x=0.5, y=1.0, offset_x=0.0, offset_y=0.0)
    without_offset = xyz_to_pan_angle(x=0.5, y=1.0)
    assert with_zero_offset == without_offset


def test_offset_x_recenters_target_directly_ahead_of_servo():
    # 서보가 레이더보다 우측 0.5m에 설치됨 → 타겟이 레이더 기준 x=0.5에 있으면
    # 서보 입장에서는 정면(0도 오프셋)에 있는 것과 같다
    angle = xyz_to_pan_angle(x=0.5, y=1.0, offset_x=0.5, offset_y=0.0)
    assert angle == 90.0


def test_offset_y_shifts_azimuth():
    baseline = xyz_to_pan_angle(x=1.0, y=2.0)
    with_offset = xyz_to_pan_angle(x=1.0, y=2.0, offset_y=1.0)
    # 서보가 레이더보다 앞쪽에 있으면 타겟까지의 상대 거리가 줄어들어
    # 같은 좌우 편차라도 각도가 더 커진다
    assert with_offset > baseline
