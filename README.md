# arda-servo

평소엔 홈 포지션(보통 강/바다 수면 방향)에 정지해 있다가, 레이더(`arda-radar`)가
낙하를 확정한 좌표를 UDP로 받으면 그 방향으로 Jetson GPIO에 연결된 팬(pan,
수평 1축) 서보 모터를 움직여 확인한다. 사람을 계속 따라다니며 추적하는
용도가 아니라, 지정된 구역을 지켜보다가 낙하 시점에만 그 지점을 보러
가는 용도다.

`arda-radar`와는 완전히 분리된 독립 프로젝트다. 두 프로세스는 로컬 UDP
소켓으로만 통신하며, 코드 레벨 의존성은 없다.

```
arda-radar (센서 읽기 → 낙하 감지) --UDP JSON--> arda-servo (좌표 수신 → 서보 각도 계산 → GPIO PWM)
```

## 동작 원리

1. 시작하면 즉시 홈 포지션(`center_deg`)으로 이동해 정지한다.
2. `arda-radar`가 낙하를 확정하면 그 좌표를 `{"x", "y", "z", "fall": true, "ts"}`
   JSON으로 UDP 전송한다 (기본 `127.0.0.1:9999`, `--no-servo-out`으로
   비활성화 가능). `fall`이 `false`인 일반 추적 좌표는 무시한다.
3. `arda-servo`가 낙하 좌표를 받으면 `azimuth = atan2(x, y)`로 수평 각도를
   구해 서보 가동 범위(`min_deg` ~ `max_deg`)로 clamp한 뒤 그 각도로
   이동한다.
4. 그 각도에서 `dwell_seconds`(기본 3초) 동안 정지한다 — 레이더의 낙하
   판정이 노이즈로 계속 반복돼도 서보가 흔들리지 않고, 서보(및 거기 달린
   열화상 카메라)가 같은 지점을 충분히 오래 봐서 사람인지 판정할 시간을
   벌어준다. dwell 중 들어오는 좌표는 전부 무시한다.
5. dwell이 끝나면 자동으로 홈 포지션(`center_deg`)에 복귀해 다음 낙하를
   기다린다.

### 열화상 추적 보정 (선택)

카메라가 서보에 고정 장착되어 있으므로, dwell 중(4단계) 열화상
(`arda-thermal-test`)이 발열 영역을 계속 감지하면 그 방향으로 서보를 더
움직여 계속 따라갈 수 있다. `arda-thermal-test`는 프레임 중심 대비 발열
위치를 정규화된 값(`offset`, -1.0~1.0)으로 `{"offset", "ts"}` JSON을
UDP로 보내고(기본 `127.0.0.1:9996`), `arda-servo`는 dwell 중에만 이를
받아 `현재 각도 + offset × thermal_udp.pan_gain_deg`로 각도를 갱신하면서
dwell을 매번 `dwell_seconds`만큼 연장한다. 보정이 더 이상 오지 않으면
(열원을 놓쳤거나 관찰이 끝나면) dwell이 자연히 만료되어 홈으로 복귀한다.
홈 포지션에서 대기 중일 때는 이 보정을 받지 않는다 — 레이더 트리거 없이
임의의 열원에 반응하지 않기 위해서다. `config/settings.yaml`의
`thermal_udp` 섹션을 통째로 지우면 이 기능은 비활성화되고 기존 동작만
남는다.

발열 영역이 사람 모양으로 확정되면 `arda-thermal-test`가
`{"confirmed": true, "vertical_offset"}`를 보낸다. `vertical_offset`은
프레임 세로 중심 대비 편차(-1.0 화면 위 ~ 1.0 화면 아래)로, `config/settings.yaml`의
`camera_geometry`(설치 높이·기울기·수직 화각)가 모두 설정돼 있으면 "카메라는
z=0 평면(지면/수면)을 보고 있다"는 가정으로 거리를 다시 계산해 좌우뿐 아니라
거리까지 반영된 최종 좌표를 로그로 남긴다. `camera_geometry`가 없거나
`vertical_offset`이 안 오면 레이더가 처음 잰 거리를 그대로 쓰고 방향만
갱신한다.

좌표계는 레이더 ROI 기준: `x`는 좌우(+가 우측, m), `y`는 센서 정면 거리(m).
`z`(높이)는 현재 팬 1축 제어에는 사용하지 않는다 — 상하(tilt) 축을
추가하면 `z`를 elevation 계산에 활용할 수 있다.

## 좌표 기준(원점) 보정

각도 계산(`arda_servo/angle.py`의 `xyz_to_pan_angle()`)은 기본적으로
**레이더 센서와 서보가 같은 위치에서, 같은 방향을 정면으로 보고 있다**고
가정한다 — 레이더 좌표계의 원점·정면 방향(azimuth 0°, `x=0`)이 서보의
정면 기준각과 일치한다는 전제다. 실제로는 두 장치가 물리적으로 떨어져
설치되는 경우가 많으므로, 그 위치 차이를 다음 두 종류의 값으로 보정한다
(전부 `config/settings.yaml`):

| 키 | 의미 | 단위 |
|---|---|---|
| `mount_offset.x` | 서보가 레이더보다 좌우로 얼마나 떨어져 설치됐는지 (레이더 우측이 +) | m |
| `mount_offset.y` | 서보가 레이더보다 전후로 얼마나 떨어져 설치됐는지 (레이더 정면 방향이 +) | m |
| `mount_offset.z` | 서보가 레이더보다 상하로 얼마나 떨어져 설치됐는지 (위쪽이 +). **팬 각도 계산에는 반영되지 않는다** (아래 설명 참고) — 시작 로그로만 확인되고, tilt 축 추가 시 사용할 예비값이다 | m |
| `servo.center_deg` | 레이더 azimuth 0°(오프셋 보정 후 정면)에 대응시킬 서보 각도. 두 장치의 정면 방향 자체가 어긋나 있으면(회전 오차) 이 값을 실측으로 조정해 보정한다 | ° (기본 90) |
| `servo.invert` | 레이더의 좌/우와 서보의 좌/우 회전 방향이 반대로 매핑되면 `true` | bool |
| `servo.min_deg` / `max_deg` | 서보가 실제로 안전하게 움직일 수 있는 각도 범위로 clamp | ° |

`mount_offset.x`/`.y`는 레이더 좌표계 기준 **수평 위치(평행이동) 차이**를
보정한다 — 타겟 좌표에서 오프셋을 뺀 뒤 그 결과로 azimuth를 계산해,
"서보 위치에서 봤을 때"의 각도로 재계산하는 방식이다. 두 장치의 정면
방향 자체가 틀어져 있는(회전) 경우는 `mount_offset`이 아니라 `center_deg`로
보정한다. 레이더와 서보가 같은 지점에 나란히, 같은 방향으로 장착돼 있다면
`mount_offset.x`/`.y`는 `0`(기본값) 그대로 두면 된다.

**`mount_offset.z`(상하 오프셋)는 팬 각도 계산과 무관하다.** 팬은 수직축을
중심으로 좌우로만 도는 회전이라, 타겟까지의 방향은 수평(x,y) 차이로만
정해지고 높이 차이의 영향을 받지 않는다 — 서보에 달린 카메라가 아래로
고정 기울여져(비모터 방식) 있어도 마찬가지다. 즉 레이더 바로 아래에
서보를 나란히(회전축 평행) 설치하는 것처럼 **z축으로만** 떨어져 있는
경우, `mount_offset.z`를 설정해도 서보가 가리키는 좌우 방향은 바뀌지
않는다. 이 값은 시작 시 로그로만 노출되며, 나중에 상하(tilt) 축을 추가할
때 elevation 각도 계산에 쓰기 위해 미리 받아두는 예비값이다.

레이더 쪽 좌표 원점(0,0,0) 자체의 정의는 `arda-radar`의 README "좌표
기준(원점) 및 유효 범위 수정" 섹션을 참고한다.

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
| `servo.center_deg` | 홈 포지션(평소 대기 각도)이자 레이더 정면(azimuth 0°)에 대응하는 서보 각도. 시작 시와 dwell 종료 후 이 각도로 이동한다 |
| `servo.invert` | 배선/장착 방향 때문에 좌우가 반대로 움직이면 `true` |
| `servo.dwell_seconds` | 낙하(`fall=true`) 좌표 수신 시 그 각도에서 정지할 시간(초), 이후 자동으로 홈 포지션 복귀. 0이면 정지 없이 즉시 복귀 |
| `thermal_udp.host` / `port` | 열화상 추적 보정 수신 UDP 바인드 주소 (기본 `0.0.0.0:9996`). 섹션을 지우면 기능 비활성화 |
| `thermal_udp.pan_gain_deg` | 보정 1건(`offset` -1.0~1.0)당 최대 회전 각도(도) |
| `camera_geometry.radar_height_m` | 레이더 설치 높이(z=0 기준, m). `arda-radar`의 `site.z`와 같은 값으로 수동으로 맞춰둘 것 |
| `camera_geometry.height_offset_from_radar_m` | 카메라가 레이더보다 높은/낮은 정도(m, 낮으면 음수). 카메라 높이 = `radar_height_m` + 이 값 |
| `camera_geometry.tilt_deg` | 수평 기준 아래로 기울어진 각도(도) — 정지 상태(프레임 세로 중심)의 앙각 |
| `camera_geometry.vertical_fov_deg` | 열화상 센서의 수직 화각(도) — 사람 확정 시 거리 역산에 사용, 섹션을 지우면 레이더 원거리를 그대로 씀 |

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
