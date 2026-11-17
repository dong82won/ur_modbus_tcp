from collections.abc import Sequence
from dataclasses import dataclass
import math
import struct
from enum import IntEnum
from ur_modbus_tcp.shared.config import CONFIG


class RegAddr(IntEnum):
    """Modbus Holding Register 주소 맵"""

    REQ_CMD = 0    # UR(Client) -> PC(Server): 1=요청, 0=대기
    READY_ACK = 1  # PC(Server) -> UR(Client): 0=대기, 1=기록완료, 2=에러
    HEARTBEAT = 2  # PC(Server) -> UR(Client): 생존 확인 카운터 (0~65535)

    # 🌟 [Eye-in-Hand 주석 수정] Robot Base 절대 좌표가 아닌 TCP 기준 '상대 좌표'임을 명시
    POSE_X = 10    # Robot TCP 기준 X 상대거리 [m]
    POSE_Y = 11    # Robot TCP 기준 Y 상대거리 [m]
    POSE_Z = 12    # Robot TCP 기준 Z 상대거리 [m]
    POSE_R = 13    # Robot TCP 기준 Roll 상대각도 [rad]
    POSE_P = 14    # Robot TCP 기준 Pitch 상대각도 [rad]
    POSE_YAW = 15  # Robot TCP 기준 Yaw 상대각도 [rad]
    TAG_ID = 16    # 검출된 tag36h11 ID (uint16)


class AckStatus(IntEnum):
    """READY_ACK 레지스터 상태 코드"""
    IDLE = 0
    READY = 1
    ERROR = 2


DATA_START_ADDR = RegAddr.POSE_X
DATA_REG_COUNT = 7   # X, Y, Z, R, P, YAW (6개) + TAG_ID (1개)

Pose6DoF = tuple[float, float, float, float, float, float]

@dataclass(frozen=True)
class TagDetection:
    tag_id: int
    # 🌟 [Eye-in-Hand 주석 수정] pose 데이터가 로봇 손목(TCP) 프레임 기준임을 명시
    pose: Pose6DoF  # (x, y, z, r, p, y) in Robot TCP frame


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

def encode_detection(detection: TagDetection, scale: float = CONFIG.scale) -> list[int]:
    """TagDetection(tag_id, 6-DoF) -> 7개의 uint16 레지스터 리스트([x, y, z, r, p, y, tag_id])"""
    if not (0 <= detection.tag_id <= 65535):
        raise ValueError(f"유효하지 않은 Tag ID 범위: {detection.tag_id}")

    pose_regs = [float_to_uint16(v, scale) for v in detection.pose]
    return [*pose_regs, int(detection.tag_id)]


def decode_detection(regs: Sequence[int], scale: float = CONFIG.scale) -> TagDetection:
    """7개의 uint16 레지스터 시퀀스 -> TagDetection 복원"""
    if len(regs) != DATA_REG_COUNT:
        raise ValueError(f"{DATA_REG_COUNT}개의 레지스터가 필요하지만 {len(regs)}개를 수신했습니다.")

    pose_vals = tuple(uint16_to_float(r, scale) for r in regs[:6])
    tag_id = int(regs[6])
    return TagDetection(tag_id=tag_id, pose=pose_vals)  # type: ignore[arg-type]
