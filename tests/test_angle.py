import math

import pytest

from arda_servo.angle import elevation_range, pan_angle_to_xyz, xyz_to_pan_angle


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


def test_pan_angle_to_xyz_round_trips_with_xyz_to_pan_angle():
    x, y = 2.0, 3.0
    angle = xyz_to_pan_angle(x, y)
    range_m = math.hypot(x, y)

    back_x, back_y = pan_angle_to_xyz(angle, range_m)

    assert back_x == pytest.approx(x)
    assert back_y == pytest.approx(y)


def test_pan_angle_to_xyz_center_angle_is_straight_ahead():
    x, y = pan_angle_to_xyz(angle_deg=90.0, range_m=5.0)
    assert abs(x) < 1e-9
    assert y == 5.0


def test_pan_angle_to_xyz_respects_invert():
    normal = pan_angle_to_xyz(angle_deg=123.69, range_m=math.hypot(2.0, 3.0))
    inverted = pan_angle_to_xyz(angle_deg=123.69, range_m=math.hypot(2.0, 3.0), invert=True)
    assert normal[0] > 0  # 오른쪽(+x)
    assert inverted[0] < 0  # invert 시 좌우가 뒤집힘


def test_pan_angle_to_xyz_round_trips_with_offset():
    x, y = 2.5, 1.5
    offset_x, offset_y = 0.5, 0.2
    angle = xyz_to_pan_angle(x, y, offset_x=offset_x, offset_y=offset_y)
    range_m = math.hypot(x - offset_x, y - offset_y)

    back_x, back_y = pan_angle_to_xyz(angle, range_m, offset_x=offset_x, offset_y=offset_y)

    assert abs(back_x - x) < 1e-9
    assert abs(back_y - y) < 1e-9


def test_elevation_range_center_offset():
    # 프레임 세로 중심(vertical_offset=0)이면 카메라 기울기 그대로가 앙각
    range_m = elevation_range(
        vertical_offset=0.0, install_height_m=1.3, camera_tilt_deg=50.0, vertical_fov_deg=35.0,
    )
    assert range_m == pytest.approx(1.3 / math.tan(math.radians(50.0)))


def test_elevation_range_bottom_of_frame_is_closer():
    # 화면 아래쪽(+1.0)은 카메라가 더 아래를 보는 셈 → 앙각이 커져 거리는 가까워짐
    center = elevation_range(0.0, 1.3, 50.0, 35.0)
    bottom = elevation_range(1.0, 1.3, 50.0, 35.0)
    assert bottom < center


def test_elevation_range_top_of_frame_is_farther():
    # 화면 위쪽(-1.0)은 카메라가 덜 아래를 보는 셈 → 앙각이 작아져 거리는 멀어짐
    center = elevation_range(0.0, 1.3, 50.0, 35.0)
    top = elevation_range(-1.0, 1.3, 50.0, 35.0)
    assert top > center


def test_elevation_range_raises_when_elevation_not_positive():
    # 기울기가 얕고 화면 위쪽이면 앙각이 0 이하로 떨어질 수 있음
    with pytest.raises(ValueError):
        elevation_range(vertical_offset=-1.0, install_height_m=1.3, camera_tilt_deg=10.0, vertical_fov_deg=35.0)
