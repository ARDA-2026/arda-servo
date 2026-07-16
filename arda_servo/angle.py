"""레이더 좌표 → 팬(pan) 서보 각도 변환."""

import math


def xyz_to_pan_angle(
    x: float,
    y: float,
    center_deg: float = 90.0,
    min_deg: float = 0.0,
    max_deg: float = 180.0,
    invert: bool = False,
    offset_x: float = 0.0,
    offset_y: float = 0.0,
) -> float:
    """레이더 좌표(X: 좌우, Y: 전방 거리, 단위 m)를 서보 각도(도)로 변환한다.

    azimuth = atan2(x, y) — 센서 정면(y축) 방향이 0도, 우측(+x)이 양수.
    center_deg를 정면 기준각으로 삼아 azimuth를 더한 뒤 가동 범위로 clamp한다.

    offset_x/offset_y(m)는 서보가 레이더 대비 설치된 위치 차이다 — 레이더
    좌표계 기준으로 "서보가 레이더에서 얼마나 떨어져 있는지"를 나타내며,
    타겟 좌표에서 이만큼을 뺀 뒤 azimuth를 계산해 서보 위치 기준으로
    보정한다. 두 장치가 같은 위치에 있으면 0(기본값)으로 둔다.
    """
    x -= offset_x
    y -= offset_y

    if y <= 0:
        y = 1e-6  # 서보 위치 기준으로 재계산한 뒤에도 방어적으로 clamp

    azimuth_deg = math.degrees(math.atan2(x, y))
    if invert:
        azimuth_deg = -azimuth_deg

    angle = center_deg + azimuth_deg
    return max(min_deg, min(max_deg, angle))
