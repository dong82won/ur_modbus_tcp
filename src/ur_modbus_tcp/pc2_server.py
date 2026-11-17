import asyncio
import logging
from pymodbus.server import ModbusTcpServer
from pymodbus.simulator import DataType, SimData, SimDevice
from ur_modbus_tcp.shared.config import CONFIG
from ur_modbus_tcp.shared.register_map import (
    AckStatus,
    POSE_START_ADDR,
    Pose6DoF,
    RegAddr,
    encode_pose,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


def create_sim_device() -> SimDevice:
  """pymodbus v4 호환 SimData / SimDevice 데이터스토어 생성 (id=0: 모든 Unit ID 응답)"""
  registers = SimData(
      address=0,
      values=[0] * CONFIG.register_count,
      datatype=DataType.REGISTERS,
  )
  return SimDevice(id=0, simdata=[registers])


def get_target_6dof() -> Pose6DoF:
  """
  실제 비전/경로 알고리즘에서 산출된 6-DoF (x, y, z, r, p, y) 반환.
  단위: x, y, z [m] / r, p, y [rad]
  (asyncio.to_thread로 호출되므로 내부에 블로킹 연산을 작성해도 안전함)
  """
  return (-0.3500, -0.4205, 0.2150, 3.1416, 0.0000, -1.5708)


async def handshake_worker(server: ModbusTcpServer) -> None:
  device_id = 0
  func_code = 3
  was_requested = False
  heartbeat = 0
  loop_count = 0

  while True:
    try:
      # 1. 주기적 하트비트 갱신 (약 0.5초마다 1씩 증가)
      loop_count += 1
      if loop_count >= int(0.5 / CONFIG.poll_interval):
        loop_count = 0
        heartbeat = (heartbeat + 1) % 65536
        await server.async_setValues(
            device_id, func_code, RegAddr.HEARTBEAT, [heartbeat]
        )

      # 2. UR 요청 레지스터 확인
      values = await server.async_getValues(
          device_id, func_code, RegAddr.REQ_CMD, count=1
      )
      if not isinstance(values, list):
        await asyncio.sleep(CONFIG.poll_interval)
        continue

      req_cmd = int(values[0])

      if req_cmd == 1 and not was_requested:
        was_requested = True
        try:
          # 비전/경로 알고리즘을 별도 스레드에서 논블로킹 실행
          pose_6dof = await asyncio.to_thread(get_target_6dof)
          regs = encode_pose(pose_6dof)

          # 데이터 먼저 기록 후 READY_ACK = 1 설정
          await server.async_setValues(
              device_id, func_code, POSE_START_ADDR, regs
          )
          await server.async_setValues(
              device_id, func_code, RegAddr.READY_ACK, [AckStatus.READY]
          )
          logger.info(f"전송 완료 | 6-DoF: {pose_6dof} -> Regs: {regs}")

        except Exception as calc_err:
          # 연산 실패 또는 범위 초과 시 서버가 죽지 않고 UR에 에러 코드(2) 전달
          logger.exception(f"6-DoF 산출/인코딩 실패: {calc_err}")
          await server.async_setValues(
              device_id, func_code, RegAddr.READY_ACK, [AckStatus.ERROR]
          )

      elif req_cmd == 0 and was_requested:
        was_requested = False
        await server.async_setValues(
            device_id, func_code, RegAddr.READY_ACK, [AckStatus.IDLE]
        )

    except asyncio.CancelledError:
      logger.info("핸드셰이크 워커가 정상 종료되었습니다.")
      break
    except Exception as loop_err:
      logger.exception(f"통신 루프 예기치 않은 오류: {loop_err}")

    await asyncio.sleep(CONFIG.poll_interval)


async def main() -> None:
  device = create_sim_device()
  server = ModbusTcpServer(
      context=device, address=(CONFIG.server_host, CONFIG.port)
  )
  task = asyncio.create_task(handshake_worker(server))
  logger.info(
      f"Ubuntu Modbus TCP 서버 시작 ({CONFIG.server_host}:{CONFIG.port})"
  )
  try:
    await server.serve_forever()
  finally:
    task.cancel()
    await server.shutdown()
    logger.info("Modbus TCP 서버가 종료되었습니다.")


def run() -> None:
  try:
    asyncio.run(main())
  except KeyboardInterrupt:
    pass


if __name__ == "__main__":
  run()
