from collections.abc import Sequence
import math
import struct
from enum import IntEnum
from ur_modbus_tcp.shared.config import CONFIG


class RegAddr(IntEnum):
  """Modbus Holding Register 주소 맵"""

  REQ_CMD = 0  # UR(Client) -> PC(Server): 1=요청, 0=대기
  READY_ACK = 1  # PC(Server) -> UR(Client): 0=대기, 1=기록완료, 2=에러
  HEARTBEAT = 2  # PC(Server) -> UR(Client): 생존 확인 카운터 (0~65535)

  POSE_X = 10  # X [m]
  POSE_Y = 11  # Y [m]
  POSE_Z = 12  # Z [m]
  POSE_R = 13  # Roll [rad]
  POSE_P = 14  # Pitch [rad]
  POSE_YAW = 15  # Yaw [rad]


class AckStatus(IntEnum):
  """READY_ACK 레지스터 상태 코드"""

  IDLE = 0
  READY = 1
  ERROR = 2


POSE_START_ADDR = RegAddr.POSE_X
POSE_REG_COUNT = 6

Pose6DoF = tuple[float, float, float, float, float, float]


def float_to_uint16(val: float, scale: float = CONFIG.scale) -> int:
  """부호 있는 실수 -> 고정소수점 int16 -> Modbus용 uint16(0~65535) 변환 (범위 검증 포함)"""
  if math.isnan(val) or math.isinf(val):
    raise ValueError(f"유효하지 않은 수치(NaN/Inf)가 입력되었습니다: {val}")

  scaled = int(round(val * scale))
  if not (-32768 <= scaled <= 32767):
    max_val = 32767 / scale
    raise ValueError(
        f"값({val})이 표현 가능 범위(±{max_val:.4f})를 초과했습니다. 단위(m, rad)를 확인하세요."
    )
  return struct.unpack("H", struct.pack("h", scaled))[0]


def uint16_to_float(raw_val: int, scale: float = CONFIG.scale) -> float:
  """Modbus용 uint16(0~65535) -> 부호 있는 실수로 복원"""
  signed = struct.unpack("h", struct.pack("H", raw_val & 0xFFFF))[0]
  return signed / scale


def encode_pose(pose: Pose6DoF, scale: float = CONFIG.scale) -> list[int]:
  """6-DoF 튜플을 6개의 uint16 레지스터 리스트로 인코딩"""
  if len(pose) != POSE_REG_COUNT:
    raise ValueError(f"6-DoF는 6개 요소가 필요하지만 {len(pose)}개가 전달되었습니다.")
  return [float_to_uint16(v, scale) for v in pose]


def decode_pose(regs: Sequence[int], scale: float = CONFIG.scale) -> Pose6DoF:
  """Sequence[int] 레지스터 시퀀스를 6-DoF 실수 튜플로 디코딩"""
  if len(regs) != POSE_REG_COUNT:
    raise ValueError(
        f"6개의 레지스터가 필요하지만 {len(regs)}개를 수신했습니다."
    )
  vals = tuple(uint16_to_float(r, scale) for r in regs)
  return vals  # type: ignore[return-value]
