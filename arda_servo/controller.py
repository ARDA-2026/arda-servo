"""좌표 수신 → 각도 변환 → 서보 구동 메인 루프."""

import time

from .angle import xyz_to_pan_angle
from .receiver import CoordReceiver
from .servo import PanServo
from .utils import get_logger

logger = get_logger(__name__)


class ServoController:
    """평소엔 홈 포지션(`center_deg`, 물 방향)에 정지해 있다가, 레이더가
    낙하를 확정(`fall=True`)한 좌표를 받을 때만 그 방향으로 움직여
    `dwell_seconds`간 머문 뒤 다시 홈으로 복귀한다. 낙하가 아닌 일반
    추적 좌표(`fall=False`)는 무시한다 — 사람을 계속 따라다니는 용도가
    아니라, 평소엔 정해진 구간(강/바다 수면 등)을 보고 있다가 낙하
    시점에만 그 지점을 확인하는 용도이기 때문이다.
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
    ):
        self._servo = servo
        self._receiver = receiver
        self._center_deg = center_deg
        self._invert = invert
        self._offset_x = offset_x
        self._offset_y = offset_y
        self._dwell_seconds = dwell_seconds
        self._dwell_until = 0.0

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
        coord = self._receiver.recv()
        now = time.time()

        if now < self._dwell_until:
            # 낙하 위치에서 머무는 중 — 열화상이 판정할 시간을 벌기 위해
            # 새 좌표가 와도 무시한다. 레이더 낙하 판정이 노이즈로
            # 반복돼도 이 dwell 동안은 서보가 흔들리지 않는다.
            if coord is not None:
                logger.debug("dwell 중 — 좌표 무시 (남은 %.1fs)", self._dwell_until - now)
            return

        if self._dwell_until:
            self._dwell_until = 0.0
            self._servo.set_angle(self._center_deg)
            logger.info("dwell 종료 — 홈 포지션(%.1f°)으로 복귀", self._center_deg)

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
        logger.info(
            "낙하 좌표 수신 — angle=%.1f°로 이동, %.1fs간 정지(dwell), 열화상 판정 대기",
            angle, self._dwell_seconds,
        )
        if self._dwell_seconds > 0:
            self._dwell_until = now + self._dwell_seconds
