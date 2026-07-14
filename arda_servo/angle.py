"""레이더 좌표 → 팬(pan) 서보 각도 변환."""

import math


def xyz_to_pan_angle(
    x: float,
    y: float,
    center_deg: float = 90.0,
    min_deg: float = 0.0,
    max_deg: float = 180.0,
    invert: bool = False,
) -> float:
    """레이더 좌표(X: 좌우, Y: 전방 거리, 단위 m)를 서보 각도(도)로 변환한다.

    azimuth = atan2(x, y) — 센서 정면(y축) 방향이 0도, 우측(+x)이 양수.
    center_deg를 정면 기준각으로 삼아 azimuth를 더한 뒤 가동 범위로 clamp한다.
    """
    if y <= 0:
        y = 1e-6  # ROI상 y >= 0.3m이라 정상 동작 중엔 발생하지 않는 방어 코드

    azimuth_deg = math.degrees(math.atan2(x, y))
    if invert:
        azimuth_deg = -azimuth_deg

    angle = center_deg + azimuth_deg
    return max(min_deg, min(max_deg, angle))
