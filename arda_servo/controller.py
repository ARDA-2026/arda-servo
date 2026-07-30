"""좌표 수신 → 각도 변환 → 서보 구동 메인 루프."""

import math
import time

from .angle import elevation_range, pan_angle_to_xyz, xyz_to_pan_angle
from .receiver import Coord, CoordReceiver
from .servo import PanServo
from .site import local_to_latlon
from .thermal_receiver import ThermalPan, ThermalPanReceiver
from .utils import get_logger

logger = get_logger(__name__)


class ServoController:
    """평소엔 홈 포지션(`center_deg`, 물 방향)에 정지해 있다가, 레이더가
    낙하를 확정(`fall=True`)한 좌표를 받을 때만 그 방향으로 움직여
    `dwell_seconds`간 머문 뒤 다시 홈으로 복귀한다. 낙하가 아닌 일반
    추적 좌표(`fall=False`)는 무시한다 — 사람을 계속 따라다니는 용도가
    아니라, 평소엔 정해진 구간(강/바다 수면 등)을 보고 있다가 낙하
    시점에만 그 지점을 확인하는 용도이기 때문이다.

    `thermal_receiver`가 주어지면, dwell 중(레이더 낙하 좌표로 이동한
    직후)에 한해 열화상(arda-thermal-test)이 보내는 발열 방향 보정값을
    받아 그 방향으로 각도를 조금씩 더 움직이고 dwell을 연장한다 — 열원이
    계속 감지되는 동안 계속 그 방향을 따라가다가, 더 이상 보정이 오지
    않으면(열원을 놓쳤거나 열화상이 안 보내면) dwell이 자연히 만료돼
    홈으로 복귀한다. 홈에서 대기 중일 때는 열화상 보정을 받지 않는다 —
    레이더 트리거 없이 임의의 열원에 반응해 움직이지 않기 위함이다.

    보정에 `vertical_offset`(프레임 세로 편차)이 함께 오고 카메라 설치
    정보(`install_height_m`/`camera_tilt_deg`/`vertical_fov_deg`)가 갖춰져
    있으면, dwell 추적 중 매 보정마다 거리·좌표를 다시 계산해 로그로 남긴다
    — 확정 순간 한 번만이 아니라 서보가 움직이는 동안 계속 위치를 갱신해서
    보여주기 위함이다.

    열화상이 사람 매칭에 계속 실패해 명시적으로 포기 신호(`give_up`)를
    보내면, dwell 만료를 기다리지 않고 그 즉시 홈으로 복귀하고 레이더
    좌표를 다시 받아들인다 — 자체 dwell 타이머만으로는 마지막 보정
    이후 추가로 dwell_seconds만큼 더 기다려야 해서 반응이 느리다.

    반대로 사람으로 확정되어 확정 신호(`confirmed`)를 받으면 마찬가지로
    즉시 홈으로 복귀하되, 이번 dwell을 시작시킨 레이더의 원좌표(x, y)와
    열화상 추적으로 바뀐 최종 좌표를 함께 로그로 남긴다. 최종 좌표의 거리는
    `install_height_m`/`camera_tilt_deg`/`vertical_fov_deg`가 모두 주어지고
    열화상이 `vertical_offset`(프레임 세로 편차)도 함께 보냈다면, "카메라는
    z=0 평면(지면/수면)을 보고 있다"는 가정으로 `elevation_range()`가 매번
    새로 역산한다 — 사람이 좌우뿐 아니라 앞뒤로도 움직였을 가능성을 반영한
    값이다. 이 정보가 없으면 레이더 원좌표의 거리를 그대로 쓰고 방향만
    최종 각도로 바꾼다(이전 방식으로 대체).

    `site_lat`/`site_lon`이 주어지면(arda-radar가 site.x/y/z 대신
    site.lat/lon/heading_deg로 낙하 위치를 GPS로 보고하는 방식과 맞추기
    위함) 이 로그의 좌표를 로컬 미터가 아니라 `local_to_latlon()`으로 변환한
    위도/경도로 남긴다. 레이더 원좌표와 열화상 추적 후 좌표 모두 이미
    "레이더 로컬 좌표계" 기준이므로(둘 다 `offset_x`/`offset_y` 보정이 끝난
    뒤의 값), 같은 `site_lat`/`site_lon`/`site_heading_deg`로 변환하면
    arda-radar의 GPS 로그와 그대로 비교할 수 있다. 주어지지 않으면 기존처럼
    로컬 좌표(m)로 남긴다.
    """

    def __init__(
        self,
        servo: PanServo,
        receiver: CoordReceiver | None = None,
        center_deg: float = 90.0,
        invert: bool = False,
        offset_x: float = 0.0,
        offset_y: float = 0.0,
        dwell_seconds: float = 3.0,
        thermal_receiver: ThermalPanReceiver | None = None,
        thermal_pan_gain_deg: float = 8.0,
        install_height_m: float | None = None,
        camera_tilt_deg: float | None = None,
        vertical_fov_deg: float | None = None,
        site_lat: float | None = None,
        site_lon: float | None = None,
        site_heading_deg: float = 0.0,
    ):
        self._servo = servo
        self._receiver = receiver
        self._center_deg = center_deg
        self._invert = invert
        self._offset_x = offset_x
        self._offset_y = offset_y
        self._dwell_seconds = dwell_seconds
        self._dwell_until = 0.0
        self._thermal_receiver = thermal_receiver
        self._thermal_pan_gain_deg = thermal_pan_gain_deg
        self._install_height_m = install_height_m
        self._camera_tilt_deg = camera_tilt_deg
        self._vertical_fov_deg = vertical_fov_deg
        self._site_lat = site_lat
        self._site_lon = site_lon
        self._site_heading_deg = site_heading_deg
        # 현재 dwell을 시작시킨 레이더 원좌표 — 열화상이 confirmed를 보낼 때
        # 최종 각도(및 가능하면 거리)를 좌표로 역산해 이 값과 비교 로그를
        # 남기기 위함.
        self._dwell_start_coord: Coord | None = None

    def run_forever(self) -> None:
        """UDP로 좌표를 수신하며 서보를 구동 (정상 운영 모드)."""
        if self._receiver is None:
            raise RuntimeError("run_forever()에는 receiver가 필요합니다 — 수동 모드는 run_manual()을 사용하세요")

        self._servo.set_angle(self._center_deg)
        logger.info("서보 제어기 시작 — 홈 포지션(%.1f°)에서 낙하 트리거 대기 중", self._center_deg)
        try:
            while True:
                self.step()
        except KeyboardInterrupt:
            logger.info("사용자 중단")
        finally:
            self._servo.close()
            self._receiver.close()
            if self._thermal_receiver is not None:
                self._thermal_receiver.close()

    def run_manual(self) -> None:
        """표준입력으로 'x y' 좌표를 직접 입력받아 서보를 구동한다.

        레이더(arda-radar)나 UDP 연결 없이 서보 하드웨어/배선만 단독으로
        테스트하기 위한 모드다.
        """
        logger.info("수동 입력 모드 시작 — 'x y' 형식으로 좌표 입력 (예: 0.5 1.2), q 입력 시 종료")
        try:
            while True:
                try:
                    line = input("x y > ").strip()
                except EOFError:
                    break

                if line.lower() in ("q", "quit", "exit"):
                    break
                if not line:
                    continue

                parts = line.split()
                if len(parts) != 2:
                    print("형식 오류 — 'x y' 두 값을 공백으로 구분해 입력하세요 (예: 0.5 1.2)")
                    continue

                try:
                    x, y = float(parts[0]), float(parts[1])
                except ValueError:
                    print("숫자로 입력해주세요 (예: 0.5 1.2)")
                    continue

                angle = xyz_to_pan_angle(
                    x, y,
                    center_deg=self._center_deg,
                    min_deg=self._servo.min_deg,
                    max_deg=self._servo.max_deg,
                    invert=self._invert,
                    offset_x=self._offset_x,
                    offset_y=self._offset_y,
                )
                self._servo.set_angle(angle)
                print(f"→ angle = {angle:.1f}°")
        except KeyboardInterrupt:
            logger.info("사용자 중단")
        finally:
            self._servo.close()

    def step(self) -> None:
        """수신 대기 1회 + (낙하 좌표면) 서보 이동. 테스트/단위 실행용으로 분리."""
        pan = self._thermal_receiver.recv() if self._thermal_receiver is not None else None
        coord = self._receiver.recv()
        now = time.time()

        if now < self._dwell_until:
            # 낙하 위치에서 머무는 중 — 열화상 보정이 오면 그 방향으로
            # 더 움직이고 dwell을 연장해 계속 추적한다. 보정이 없으면
            # 레이더 낙하 판정이 노이즈로 반복돼도 서보가 흔들리지 않도록
            # 새 좌표는 무시한다.
            if pan is not None and (pan.give_up or pan.confirmed):
                self._end_tracking(pan)
            elif pan is not None:
                self._apply_thermal_pan(pan.offset)
                self._dwell_until = now + self._dwell_seconds

                coord = self._dwell_start_coord
                if coord is not None:
                    range_m, range_note = self._resolve_range(coord, pan.vertical_offset)
                    position = self._describe_position(self._servo.angle, range_m)
                    logger.info(
                        "열화상 보정 반영 — offset=%.2f → angle=%.1f°, 추적 연장 | "
                        "현재 %s (거리 %.2fm, %s)",
                        pan.offset, self._servo.angle, position, range_m, range_note,
                    )
                else:
                    logger.info(
                        "열화상 보정 반영 — offset=%.2f → angle=%.1f°, 추적 연장",
                        pan.offset, self._servo.angle,
                    )
            elif coord is not None:
                logger.debug("dwell 중 — 좌표 무시 (남은 %.1fs)", self._dwell_until - now)
            return

        if self._dwell_until:
            final_angle = self._servo.angle
            self._dwell_until = 0.0
            self._servo.set_angle(self._center_deg)
            logger.info(
                "dwell 시간 초과로 종료 — 최종 각도=%.1f° → 홈 포지션(%.1f°)으로 복귀",
                final_angle, self._center_deg,
            )
            self._dwell_start_coord = None

        if coord is None or not coord.fall:
            # 낙하가 아닌 일반 추적 좌표는 무시한다 — 평소엔 홈 포지션에
            # 고정해 지정된 구역(강/바다 등)을 보고 있어야 하기 때문이다.
            return

        angle = xyz_to_pan_angle(
            coord.x,
            coord.y,
            center_deg=self._center_deg,
            min_deg=self._servo.min_deg,
            max_deg=self._servo.max_deg,
            invert=self._invert,
            offset_x=self._offset_x,
            offset_y=self._offset_y,
        )
        self._servo.set_angle(angle)
        self._dwell_start_coord = coord
        logger.info(
            "낙하 좌표 수신 — x=%.2f y=%.2f → angle=%.1f°로 이동, %.1fs간 정지(dwell), 열화상 판정 대기",
            coord.x, coord.y, angle, self._dwell_seconds,
        )
        if self._dwell_seconds > 0:
            self._dwell_until = now + self._dwell_seconds

    def _apply_thermal_pan(self, offset: float) -> None:
        """열화상이 보낸 정규화 편차(-1.0~1.0)만큼 현재 각도에서 더 회전함."""
        if self._invert:
            offset = -offset
        current = self._servo.angle if self._servo.angle is not None else self._center_deg
        new_angle = current + offset * self._thermal_pan_gain_deg
        self._servo.set_angle(new_angle)

    def _end_tracking(self, pan: ThermalPan) -> None:
        """열화상의 give_up/confirmed 신호를 받아 dwell을 즉시 끝내고 홈으로 복귀함."""
        final_angle = self._servo.angle
        self._dwell_until = 0.0
        self._servo.set_angle(self._center_deg)

        if pan.confirmed:
            coord = self._dwell_start_coord
            if coord is not None:
                range_m, range_note = self._resolve_range(coord, pan.vertical_offset)
                orig_position = self._describe_xy(coord.x, coord.y)
                final_position = self._describe_position(final_angle, range_m)
                position_word = "위치" if self._site_lat is not None else "좌표"
                logger.warning(
                    "열화상 사람 확정 — 레이더 원%s %s → 열화상 추적 후 %s %s "
                    "(거리 %.2fm, %s) → 홈(%.1f°) 복귀",
                    position_word, orig_position, position_word, final_position,
                    range_m, range_note, self._center_deg,
                )
            else:
                logger.warning(
                    "열화상 사람 확정 — 레이더 원좌표 없음, 최종 각도=%.1f° → 홈(%.1f°) 복귀",
                    final_angle, self._center_deg,
                )
        else:
            logger.info(
                "열화상이 추적을 포기함 — 즉시 홈 포지션(%.1f°)으로 복귀, 레이더 트리거 재개",
                self._center_deg,
            )

        self._dwell_start_coord = None

    def _resolve_range(self, coord: Coord, vertical_offset: float | None) -> tuple[float, str]:
        """현재 거리를 구함. 가능하면 카메라 설치 정보로 z=0 기준 역산하고,
        정보가 부족하면 레이더 원좌표의 거리를 그대로 쓴다. (거리, 설명) 반환.
        확정 시점뿐 아니라 dwell 추적 중 매 보정마다도 호출된다."""
        geometry_ready = (
            self._install_height_m is not None
            and self._camera_tilt_deg is not None
            and self._vertical_fov_deg is not None
        )
        radar_range_m = math.hypot(coord.x - self._offset_x, coord.y - self._offset_y)

        if not geometry_ready or vertical_offset is None:
            return radar_range_m, "레이더 원거리 유지, 방향만 갱신"

        try:
            range_m = elevation_range(
                vertical_offset, self._install_height_m, self._camera_tilt_deg, self._vertical_fov_deg,
            )
            return range_m, "z=0 평면 기준 열화상 세로 위치로 거리 역산"
        except ValueError as e:
            logger.warning("거리 역산 실패(%s) — 레이더 원거리로 대체", e)
            return radar_range_m, "레이더 원거리로 대체"

    def _describe_xy(self, x: float, y: float) -> str:
        """로컬 (x, y)를 site 설정이 있으면 위경도, 없으면 로컬 좌표 문자열로 변환함."""
        if self._site_lat is not None and self._site_lon is not None:
            lat, lon = local_to_latlon(x, y, self._site_lat, self._site_lon, self._site_heading_deg)
            return f"lat={lat:.6f} lon={lon:.6f}"
        return f"x={x:.2f} y={y:.2f}"

    def _describe_position(self, angle: float, range_m: float) -> str:
        """서보 각도·거리로 좌표를 구해 _describe_xy()로 문자열화함."""
        x, y = pan_angle_to_xyz(
            angle, range_m,
            center_deg=self._center_deg, invert=self._invert,
            offset_x=self._offset_x, offset_y=self._offset_y,
        )
        return self._describe_xy(x, y)
