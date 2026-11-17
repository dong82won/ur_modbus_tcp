# UR Modbus TCP (`ur_modbus_tcp`)

Universal Robots(UR) 제어기(Client)와 외부 PC(Server) 간 **6-DoF(x, y, z, r, p, y) 좌표 데이터 송수신 및 연결 상태 감시(Heartbeat)** 를 위한 Python 비동기 Modbus TCP 통신 프로젝트입니다.

비전/경로 생성 알고리즘이 탑재된 PC 서버(`pc2_server.py`)와 로봇 제어기를 모사한 테스트 클라이언트(`pc1_client.py`)로 구성되어 있으며, 데이터 정합성을 보장하는 핸드셰이크(Handshake) 프로토콜과 고정소수점 실수 인코딩을 구현합니다.

---

## 1. 프로젝트 구조

Git 업로드 시 빌드 캐시(`__pycache__`)를 제외한 핵심 소스 구조는 다음과 같습니다.

```text
ur_modbus_TCP/
├── pyproject.toml               # 프로젝트 메타데이터 및 의존성 정의
├── uv.lock                      # uv 패키지 매니저 잠금 파일
├── README.md                    # 프로젝트 문서
└── src/
    └── ur_modbus_tcp/
        ├── __init__.py
        ├── pc1_client.py        # Modbus TCP 클라이언트 (UR 로봇 시뮬레이터 및 하트비트 검증)
        ├── pc2_server.py        # Modbus TCP 서버 (6-DoF 좌표 제공 및 핸드셰이크 처리)
        └── shared/
            ├── __init__.py
            ├── config.py        # 통신 포트, 폴링 주기, 스케일 등 전역 설정
            └── register_map.py  # Holding Register 주소 맵 및 실수-정수 인코딩/디코딩
```
---

## 2. Modbus Holding Register 맵 및 프로토콜

### 레지스터 주소 맵 (`RegAddr`)

| 주소 (Address) | 이름 | 방향 | 타입 / 범위 | 설명 |
| :---: | :--- | :---: | :---: | :--- |
| `0` | `REQ_CMD` | Client → Server | `uint16` (`0`, `1`) | `1`: 6-DoF 좌표 요청, `0`: 대기(초기화) |
| `1` | `READY_ACK` | Server → Client | `uint16` (`0`, `1`, `2`) | `0`: 대기(IDLE), `1`: 기록 완료(READY), `2`: 연산 오류(ERROR) |
| `2` | `HEARTBEAT` | Server → Client | `uint16` (`0` ~ `65535`) | 약 0.5초마다 1씩 증가하는 서버 생존 카운터 |
| `10` ~ `12` | `POSE_X`, `Y`, `Z` | Server → Client | `int16` → `uint16` | 직교 좌표 위치 `[m]` (스케일 ×10,000, 범위 `±3.2767 m`) |
| `13` ~ `15` | `POSE_R`, `P`, `YAW` | Server → Client | `int16` → `uint16` | 회전 자세 `[rad]` (스케일 ×10,000, 범위 `±3.2767 rad`) |

### 핸드셰이크(Handshake) 동작 시퀀스

1. **요청 트리거**: 클라이언트가 `REQ_CMD(0)`에 `1`을 기록합니다.
2. **좌표 산출 및 기록**: 서버가 `REQ_CMD == 1`을 감지하면 6-DoF 좌표를 계산하여 `10~15`번 레지스터에 **먼저 기록**한 뒤, `READY_ACK(1)`을 `1`로 설정합니다. (연산 실패 시 `2` 기록)
3. **데이터 수신**: 클라이언트가 `READY_ACK == 1`을 확인하고 `10~15`번 레지스터 6개를 일괄 읽어 실수로 복원합니다.
4. **상태 초기화**: 클라이언트가 `REQ_CMD(0)`을 `0`으로 되돌리면, 서버가 `READY_ACK(1)`을 `0`으로 초기화하여 다음 요청을 대기합니다.

---

## 3. 소스 코드 상세 및 주요 문법 설명

### ① `src/ur_modbus_tcp/shared/config.py`
통신 환경과 데이터 변환에 필요한 상수를 중앙 관리합니다.
* **`@dataclass(frozen=True)`**:
불변(Immutable) 데이터 클래스를 생성하여 런타임 중 설정값이 의도치 않게 변경되는 것을 방지합니다.
* **비특권 포트(`5020`) 사용**:
Linux/Ubuntu 환경에서 루트(`sudo`) 권한 없이 서버를 실행할 수 있도록 표준 포트(`502`) 대신 `5020`번 포트를 기본값으로 사용합니다.
* **고정소수점 스케일(`scale = 10000.0`)**:
소수점 넷째 자리(`0.1 mm` / `0.0001 rad`) 정밀도를 유지하기 위한 배율입니다.

### ② `src/ur_modbus_tcp/shared/register_map.py`
16비트 부호 없는 정수(`uint16`)만 지원하는 Modbus 레지스터에 부호 있는 실수(`float`)를 안전하게 담기 위한 변환 모듈입니다.

* **`IntEnum` (`RegAddr`, `AckStatus`)**: 정수형 열거형을 상속받아 레지스터 주소와 상태 코드를 매직 넘버 없이 가독성 높게 표현하며, `pymodbus` 함수의 `int` 인자로 바로 전달할 수 있습니다.
* **`struct.pack` / `struct.unpack` 비트 변환**:
  * Modbus 레지스터는 `0 ~ 65535`(`uint16`) 범위만 취급하므로 음수를 직접 저장할 수 없습니다.
  * `struct.unpack("H", struct.pack("h", scaled))[0]`를 통해 부호 있는 16비트 정수(`h`, `-32768 ~ 32767`)의 2의 보수(Two's complement) 바이너리 패턴을 그대로 유지한 채 부호 없는 16비트 정수(`H`)로 변환합니다.
* **방어적 유효성 검증**: `math.isnan()`, `math.isinf()` 및 `±3.2767` 범위 초과 여부를 사전 검증하여 잘못된 좌표가 로봇에 전송되는 것을 차단합니다.
* **타입 힌팅 (`Pose6DoF`, `Sequence[int]`)**: 6개 요소의 튜플 타입 별칭과 범용 시퀀스 타입을 명시해 정적 타입 검사 안정성을 높였습니다.

### ③ `src/ur_modbus_tcp/pc2_server.py`
외부 PC(비전/제어 서버) 역할을 수행하는 비동기 Modbus TCP 서버입니다.

* **`pymodbus` v4 데이터스토어 (`SimData`, `SimDevice`)**: `SimDevice(id=0, ...)`로 초기화하여 클라이언트가 어떤 Unit ID(Slave ID)로 요청하더라도 응답할 수 있도록 구성했습니다.
* **논블로킹 연산 오프로딩 (`asyncio.to_thread`)**: `get_target_6dof()` 내부에 무거운 비전 처리나 역기구학 연산이 추가되더라도, 별도 워커 스레드에서 실행하여 하트비트 갱신 및 통신 이벤트 루프가 멈추지 않도록 설계했습니다.
* **에지 검출(`was_requested`) 및 Race Condition 방지**: `req_cmd == 1`이 유지되는 동안 중복 연산을 막고, 좌표 데이터(`10~15`) 기록 완료 후 상태 플래그(`READY_ACK = 1`)를 마지막에 갱신하여 클라이언트가 미완성 데이터를 읽는 현상을 방지합니다.

### ④ `src/ur_modbus_tcp/pc1_client.py`
UR 로봇 제어기를 모사하여 좌표 요청 및 서버 생존 여부를 검증하는 테스트 클라이언트입니다.
* **동시성 태스크 (`asyncio.create_task`)**: 메인 루프에서 6-DoF 좌표를 3회 요청하는 동안, 백그라운드 태스크(`verify_heartbeat`)가 병렬로 실행되며 하트비트 레지스터를 감시합니다.
* **단조 증가 시계 (`time.monotonic`)**: OS 시스템 시간 변경의 영향을 받지 않는 단조 시계를 사용하여 하트비트 갱신 주기(`~0.5s`)와 타임아웃(`1.5s`, 주기의 3배)을 정밀하게 측정합니다.
* **예외 격리**: 서버가 강제 종료(`Ctrl+C`)되어 `ModbusException`이나 `OSError`가 발생하더라도 즉시 크래시하지 않고, 지정된 타임아웃(`1.5s`) 경과 후 서버 단절 경고를 출력하도록 구현했습니다.

---

## 4. 설치 및 실행 방법

### 사전 요구사항
* Python `3.12` 이상
* [`uv`](https://docs.astral.sh/uv/) 패키지 매니저

### 1) 의존성 설치
프로젝트 루트 디렉터리(`ur_modbus_TCP/`)에서 가상환경 구성 및 패키지를 설치합니다.

```bash
uv sync
```

### 2) 서버 실행 (터미널 1)
`src` 레이아웃 패키지 경로 인식을 위해 모듈(`-m`) 방식으로 실행합니다.

```bash
uv run python -m ur_modbus_tcp.pc2_server
```
* 실행 로그: `Ubuntu Modbus TCP 서버 시작 (0.0.0.0:5020)`

### 3) 클라이언트 실행 및 테스트 (터미널 2)
새 터미널을 열고 클라이언트를 실행합니다.

```bash
uv run python -m ur_modbus_tcp.pc1_client
```
* **정상 동작**: 서버 연결 후 6-DoF 좌표 `(-0.35, -0.4205, 0.215, 3.1416, 0.0, -1.5708)`를 3회 수신하고 하트비트 갱신 주기를 출력합니다.
* **타임아웃 검증**: 3회 테스트 완료 후 클라이언트 대기 상태에서 서버 터미널을 `Ctrl+C`로 종료하면, 약 1.5초 뒤 `[경고] 하트비트 정지 감지!` 메시지가 출력됩니다.
