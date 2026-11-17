from dataclasses import dataclass


@dataclass(frozen=True)
class ModbusConfig:
  # Modbus TCP 통신 설정
  server_host: str = "0.0.0.0"  # 서버 바인딩 IP (모든 인터페이스 허용)
  target_ip: str = "127.0.0.1"  # 테스트 클라이언트가 접속할 서버 IP
  port: int = 5020  # 비특권 TCP 포트 (502 대신 사용)

  register_count: int = 100  # Holding Register 전체 할당 크기
  poll_interval: float = 0.01  # 핸드셰이크 감시 주기 (10ms)
  scale: float = 10000.0  # 고정소수점 스케일 (소수점 4자리 보존)


  # Intel RealSense 카메라 설정
  cam_width: int = 640
  cam_height: int = 480
  cam_fps: int = 30

  # AprilTag (tag36h11) 설정
  tag_family: str = "tag36h11"
  tag_size_m: float = 0.048          # 실제 태그 한 변의 길이 [m] (예: 48mm = 0.048)
  min_decision_margin: float = 35.0  # 오검출 필터링 임계값 (낮으면 노이즈 제거 강화)

  # OpenCV 실시간 화면 표시 여부 (모니터 없는 운영 환경에서는 False로 변경)
  show_preview: bool = True


CONFIG = ModbusConfig()
