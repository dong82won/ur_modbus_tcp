from dataclasses import dataclass


@dataclass(frozen=True)
class ModbusConfig:
  server_host: str = "0.0.0.0"  # 서버 바인딩 IP (모든 인터페이스 허용)
  target_ip: str = "127.0.0.1"  # 테스트 클라이언트가 접속할 서버 IP
  port: int = 5020  # 비특권 TCP 포트 (502 대신 사용)

  register_count: int = 100  # Holding Register 전체 할당 크기
  poll_interval: float = 0.01  # 핸드셰이크 감시 주기 (10ms)
  scale: float = 10000.0  # 고정소수점 스케일 (소수점 4자리 보존)


CONFIG = ModbusConfig()
