"""arda-servo 엔트리포인트 — 레이더 좌표 기반 팬(pan) 서보 제어."""

import argparse
from pathlib import Path

import yaml

from arda_servo.controller import ServoController
from arda_servo.receiver import CoordReceiver
from arda_servo.servo import PanServo
from arda_servo.thermal_receiver import ThermalPanReceiver
from arda_servo.utils import get_logger

logger = get_logger(__name__)

DEFAULT_CONFIG = Path("config/settings.yaml")


def load_config(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ARDA Servo — 레이더 좌표 기반 팬 서보 제어")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="설정 파일 경로")
    parser.add_argument("--simulate", action="store_true", help="GPIO 없이 시뮬레이션 모드로 실행")
    parser.add_argument(
        "--manual", action="store_true",
        help="UDP/레이더 없이 표준입력으로 'x y' 좌표를 직접 입력해 서보만 단독 테스트",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(Path(args.config))
    servo_cfg = cfg["servo"]

    servo = PanServo(
        pin=servo_cfg["gpio_pin"],
        min_deg=servo_cfg.get("min_deg", 0.0),
        max_deg=servo_cfg.get("max_deg", 180.0),
        simulate=args.simulate,
    )

    offset_cfg = cfg.get("mount_offset", {})
    offset_x = offset_cfg.get("x", 0.0)
    offset_y = offset_cfg.get("y", 0.0)
    offset_z = offset_cfg.get("z", 0.0)
    if offset_z:
        logger.info(
            "mount_offset.z=%.2fm 설정됨 — 팬(pan) 각도 계산에는 미반영 "
            "(높이 차이는 좌우 회전에 영향 없음, tilt 축 추가 시 사용 예정)",
            offset_z,
        )

    if args.manual:
        controller = ServoController(
            servo,
            center_deg=servo_cfg.get("center_deg", 90.0),
            invert=servo_cfg.get("invert", False),
            offset_x=offset_x,
            offset_y=offset_y,
        )
        controller.run_manual()
        return

    udp_cfg = cfg["udp"]
    receiver = CoordReceiver(
        host=udp_cfg["host"],
        port=udp_cfg["port"],
        timeout=udp_cfg.get("timeout", 0.5),
    )

    thermal_cfg = cfg.get("thermal_udp")
    thermal_receiver = None
    if thermal_cfg:
        thermal_receiver = ThermalPanReceiver(
            host=thermal_cfg.get("host", "0.0.0.0"),
            port=thermal_cfg.get("port", 9996),
            timeout=thermal_cfg.get("timeout", 0.01),
        )
        logger.info(
            "열화상 추적 보정 수신 활성화 — UDP %s:%d (gain=%.1f°)",
            thermal_cfg.get("host", "0.0.0.0"), thermal_cfg.get("port", 9996),
            thermal_cfg.get("pan_gain_deg", 8.0),
        )

    geometry_cfg = cfg.get("camera_geometry")
    install_height_m = camera_tilt_deg = vertical_fov_deg = None
    if geometry_cfg:
        radar_height_m = geometry_cfg.get("radar_height_m")
        height_offset_from_radar_m = geometry_cfg.get("height_offset_from_radar_m", 0.0)
        install_height_m = radar_height_m + height_offset_from_radar_m
        camera_tilt_deg = geometry_cfg.get("tilt_deg")
        vertical_fov_deg = geometry_cfg.get("vertical_fov_deg")
        logger.info(
            "카메라 설치 기하 활성화 — 레이더 높이=%.2fm %+.2fm → 카메라 높이=%.2fm, "
            "기울기=%.1f°, 수직화각=%.1f° (사람 확정 시 z=0 평면 기준 거리 역산에 사용)",
            radar_height_m, height_offset_from_radar_m, install_height_m,
            camera_tilt_deg, vertical_fov_deg,
        )

    controller = ServoController(
        servo,
        receiver,
        center_deg=servo_cfg.get("center_deg", 90.0),
        invert=servo_cfg.get("invert", False),
        offset_x=offset_x,
        offset_y=offset_y,
        dwell_seconds=servo_cfg.get("dwell_seconds", 10.0),
        thermal_receiver=thermal_receiver,
        thermal_pan_gain_deg=thermal_cfg.get("pan_gain_deg", 8.0) if thermal_cfg else 8.0,
        install_height_m=install_height_m,
        camera_tilt_deg=camera_tilt_deg,
        vertical_fov_deg=vertical_fov_deg,
    )

    logger.info("ARDA Servo 시작 — UDP %s:%d 수신 대기", udp_cfg["host"], udp_cfg["port"])
    controller.run_forever()


if __name__ == "__main__":
    main()
