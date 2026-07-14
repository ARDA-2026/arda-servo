"""좌표 수신 → 각도 변환 → 서보 구동 메인 루프."""

import time

from .angle import xyz_to_pan_angle
from .receiver import CoordReceiver
from .servo import PanServo
from .utils import get_logger

logger = get_logger(__name__)


class ServoController:
    def __init__(
        self,
        servo: PanServo,
        receiver: CoordReceiver | None = None,
        center_deg: float = 90.0,
        invert: bool = False,
        stale_timeout: float = 2.0,
    ):
        self._servo = servo
        self._receiver = receiver
        self._center_deg = center_deg
        self._invert = invert
        self._stale_timeout = stale_timeout
        self._last_update = 0.0
        self._stale_logged = False

    def run_forever(self) -> None:
        """UDP로 좌표를 수신하며 서보를 구동 (정상 운영 모드)."""
        if self._receiver is None:
            raise RuntimeError("run_forever()에는 receiver가 필요합니다 — 수동 모드는 run_manual()을 사용하세요")

        logger.info("서보 제어기 시작 — 좌표 대기 중")
        try:
            while True:
                self.step()
        except KeyboardInterrupt:
            logger.info("사용자 중단")
        finally:
            self._servo.close()
            self._receiver.close()

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
                )
                self._servo.set_angle(angle)
                print(f"→ angle = {angle:.1f}°")
        except KeyboardInterrupt:
            logger.info("사용자 중단")
        finally:
            self._servo.close()

    def step(self) -> None:
        """수신 대기 1회 + (좌표가 있으면) 서보 각도 갱신. 테스트/단위 실행용으로 분리."""
        coord = self._receiver.recv()
        now = time.time()

        if coord is not None:
            angle = xyz_to_pan_angle(
                coord.x,
                coord.y,
                center_deg=self._center_deg,
                min_deg=self._servo.min_deg,
                max_deg=self._servo.max_deg,
                invert=self._invert,
            )
            self._servo.set_angle(angle)
            self._last_update = now
            self._stale_logged = False
            logger.debug("좌표 수신 x=%.2f y=%.2f → angle=%.1f°", coord.x, coord.y, angle)
        elif (
            self._last_update
            and not self._stale_logged
            and (now - self._last_update) > self._stale_timeout
        ):
            logger.warning("좌표 미수신 %.1fs 경과 — 마지막 각도(%.1f°) 유지", now - self._last_update, self._servo.angle)
            self._stale_logged = True
