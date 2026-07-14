"""Jetson GPIO 기반 팬(pan) 서보 제어."""

from .utils import get_logger

logger = get_logger(__name__)

try:
    import Jetson.GPIO as GPIO
    _HAS_GPIO = True
except Exception as e:  # noqa: BLE001 — Jetson.GPIO는 미지원 보드에서 import 시점에 일반 Exception을 던짐
    GPIO = None
    _HAS_GPIO = False
    _GPIO_IMPORT_ERROR = e

PWM_FREQ_HZ = 50.0    # 표준 아날로그 서보 PWM 주파수
MIN_PULSE_MS = 0.5    # min_deg에 대응하는 펄스폭
MAX_PULSE_MS = 2.5    # max_deg에 대응하는 펄스폭


class PanServo:
    """단일 축(pan) 서보 모터 제어기. Jetson.GPIO 소프트웨어 PWM 사용.

    Jetson.GPIO를 로드할 수 없는 환경(보드 미인식, 권한 부족 등)에서는
    자동으로 시뮬레이션 모드로 동작해 상위 로직(좌표 수신·각도 계산)을
    하드웨어 없이도 개발/테스트할 수 있게 한다.
    """

    def __init__(
        self,
        pin: int,
        min_deg: float = 0.0,
        max_deg: float = 180.0,
        min_pulse_ms: float = MIN_PULSE_MS,
        max_pulse_ms: float = MAX_PULSE_MS,
        simulate: bool = False,
    ):
        self._pin = pin
        self._min_deg = min_deg
        self._max_deg = max_deg
        self._min_pulse_ms = min_pulse_ms
        self._max_pulse_ms = max_pulse_ms
        self._angle: float | None = None
        self._pwm = None

        self._simulate = simulate or not _HAS_GPIO
        if not _HAS_GPIO and not simulate:
            logger.warning("Jetson.GPIO 로드 실패(%s) — 시뮬레이션 모드로 전환", _GPIO_IMPORT_ERROR)
        if self._simulate:
            if simulate:
                logger.info("시뮬레이션 모드로 실행 — 실제 서보는 움직이지 않음")
            return

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pin, GPIO.OUT)
        self._pwm = GPIO.PWM(self._pin, PWM_FREQ_HZ)
        self._pwm.start(0)

    @property
    def min_deg(self) -> float:
        return self._min_deg

    @property
    def max_deg(self) -> float:
        return self._max_deg

    @property
    def angle(self) -> float | None:
        return self._angle

    def set_angle(self, angle_deg: float) -> None:
        angle_deg = max(self._min_deg, min(self._max_deg, angle_deg))
        self._angle = angle_deg

        if self._simulate:
            logger.debug("SIM set_angle=%.1f°", angle_deg)
            return

        span = self._max_deg - self._min_deg
        pulse_ms = self._min_pulse_ms + (angle_deg - self._min_deg) / span * (
            self._max_pulse_ms - self._min_pulse_ms
        )
        duty = pulse_ms / (1000.0 / PWM_FREQ_HZ) * 100.0
        self._pwm.ChangeDutyCycle(duty)

    def close(self) -> None:
        if self._simulate:
            return
        self._pwm.stop()
        GPIO.cleanup(self._pin)
