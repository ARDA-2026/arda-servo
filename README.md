# arda-servo

레이더(`arda-radar`)가 탐지한 타겟 좌표를 UDP로 받아, 그 방향을 향하도록
Jetson GPIO에 연결된 팬(pan, 수평 1축) 서보 모터를 구동한다.

`arda-radar`와는 완전히 분리된 독립 프로젝트다. 두 프로세스는 로컬 UDP
소켓으로만 통신하며, 코드 레벨 의존성은 없다.

```
arda-radar (센서 읽기 → 낙하 감지) --UDP JSON--> arda-servo (좌표 수신 → 서보 각도 계산 → GPIO PWM)
```

## 동작 원리

1. `arda-radar`가 매 프레임 탐지된 타겟의 평활화된 무게중심 `(x, y, z)`를
   `{"x", "y", "z", "fall", "ts"}` JSON으로 UDP 전송한다 (기본
   `127.0.0.1:9999`, `--no-servo-out`으로 비활성화 가능).
2. `arda-servo`가 그 좌표를 받아 `azimuth = atan2(x, y)`로 수평 각도를
   구하고, 서보 가동 범위(`min_deg` ~ `max_deg`)로 clamp한 뒤 GPIO PWM
   듀티비를 갱신한다.
3. 좌표가 `stale_timeout`(기본 2초) 이상 끊기면 마지막 각도를 유지한 채
   경고 로그만 남긴다.

좌표계는 레이더 ROI 기준: `x`는 좌우(+가 우측, m), `y`는 센서 정면 거리(m).
`z`(높이)는 현재 팬 1축 제어에는 사용하지 않는다 — 상하(tilt) 축을
추가하면 `z`를 elevation 계산에 활용할 수 있다.

## 하드웨어 배선

- 서보 신호선 → Jetson 40핀 헤더의 PWM 지원 핀 (기본 설정: **물리 핀
  33**, `config/settings.yaml`의 `servo.gpio_pin`으로 변경 가능. BOARD
  넘버링 기준이며 보드 모델별로 어떤 핀이 하드웨어 PWM을 지원하는지
  `sudo jetson-io.py` 또는 NVIDIA 공식 핀아웃 문서로 확인할 것)
- 서보 GND → Jetson GND 핀
- 서보 전원(V+) → **Jetson 5V 핀에서 직접 끌어오지 말 것.** 서보 기동/정지
  시 순간 전류가 보드를 리셋시킬 수 있다. 별도 5V 외부 전원을 사용하고
  GND만 Jetson과 공통으로 묶는다.

## 설치 및 실행

```bash
cd arda-servo
uv sync
uv run python main.py                 # 실제 GPIO로 서보 구동 (UDP로 좌표 수신)
uv run python main.py --simulate      # 하드웨어 없이 각도 계산만 로그로 확인

# 레이더/UDP 없이 서보 배선·가동범위만 단독 테스트 (좌표를 직접 입력)
uv run python main.py --manual
uv run python main.py --manual --simulate   # 서보도 없이 각도 계산만 확인
```

`--manual` 모드에서는 `x y` 형식으로(공백 구분, 예: `0.5 1.2`) 좌표를 입력할
때마다 즉시 해당 방향으로 서보가 움직인다. `q`를 입력하면 종료한다. UDP
소켓을 아예 열지 않으므로 `arda-radar` 없이도 서보 하드웨어(배선, 가동범위,
`invert` 설정 등)를 바로 확인할 수 있다.

`arda-radar` 쪽은 좌표 전송이 기본 활성화되어 있다 (같은 보드에서 실행 시
추가 설정 불필요):

```bash
cd arda-radar
uv run python main.py
```

두 프로젝트를 서로 다른 장비에서 실행하려면 `arda-radar`는
`--servo-host <arda-servo IP>`로, `arda-servo`는
`config/settings.yaml`의 `udp.host`를 `0.0.0.0`(전체 인터페이스 수신)으로
맞춘다.

## 설정 (`config/settings.yaml`)

| 키 | 설명 |
|---|---|
| `udp.host` / `udp.port` | 좌표 수신 UDP 바인드 주소 (기본 `0.0.0.0:9999`) |
| `servo.gpio_pin` | 서보 신호선이 연결된 물리 핀 번호 (BOARD 모드) |
| `servo.min_deg` / `max_deg` | 서보 가동 각도 범위 |
| `servo.center_deg` | 레이더 정면(azimuth 0°)에 대응하는 서보 각도 |
| `servo.invert` | 배선/장착 방향 때문에 좌우가 반대로 움직이면 `true` |
| `servo.stale_timeout` | 이 시간(초) 이상 좌표가 끊기면 경고 로그 |

## 테스트

```bash
uv run pytest
```

좌표→각도 변환(`angle.py`)과 UDP 수신 파싱(`receiver.py`)은 하드웨어 없이
순수 로직으로 테스트된다. `servo.py`는 `Jetson.GPIO`를 로드할 수 없는
환경(개발 PC, CI 등)에서 자동으로 시뮬레이션 모드로 폴백하므로, 실제 보드가
없어도 `PanServo(pin=.., simulate=True)`로 상위 로직을 검증할 수 있다.

## Jetson.GPIO 트러블슈팅

**"Could not determine Jetson model" 에러가 나는 경우**: 이 보드(Jetson
Orin Nano Super 개발자 키트)는 device-tree compatible 문자열이
`nvidia,p3768-0000+p3767-0005-super`인데, `apt`로 설치된
`python3-jetson-gpio` 2.1.7은 이 보드를 인식하지 못한다. 이 프로젝트의
`pyproject.toml`은 PyPI의 `Jetson.GPIO`(2.1.12+, `uv sync`가 자동 설치)를
쓰도록 되어 있고, 이 버전은 정상 인식하므로 **`uv run`으로 실행하는 한
별도 조치가 필요 없다.** 시스템 파이썬으로 직접 `import Jetson.GPIO`를
할 경우에만 위 문제가 재현된다.

GPIO 접근에는 `gpio` 그룹 권한이 필요하다 (`/dev/gpiochip*`가
`root:gpio`). 실행 계정이 `gpio` 그룹에 속해 있는지 `groups` 명령으로
확인한다.
