import asyncio
import time
from pymodbus.client import AsyncModbusTcpClient
from pymodbus.exceptions import ModbusException
from ur_modbus_tcp.shared.config import CONFIG
from ur_modbus_tcp.shared.register_map import (
    AckStatus,
    DATA_REG_COUNT,
    DATA_START_ADDR,
    RegAddr,
    decode_detection,
)

async def cleanup_and_wait_idle(client: AsyncModbusTcpClient) -> None:
    """요청 종료 후 REQ_CMD를 0으로 초기화하고 서버가 IDLE 상태로 돌아올 때까지 대기합니다."""
    await client.write_register(address=RegAddr.REQ_CMD, value=0, device_id=0)
    while True:
        ack_res = await client.read_holding_registers(address=RegAddr.READY_ACK, count=1, device_id=0)
        if not ack_res.isError() and ack_res.registers[0] == AckStatus.IDLE:
            break
        await asyncio.sleep(CONFIG.poll_interval)
    print("[CLIENT] req_cmd = 0 초기화 및 서버 IDLE 상태 복귀 확인 완료\n")

async def request_tag_from_server(
    client: AsyncModbusTcpClient, timeout_sec: float = 2.0
) -> None:
    # 1. 실제 데이터 요청 트리거 (slave=0 명시)
    await client.write_register(address=RegAddr.REQ_CMD, value=1, device_id=0)
    print("[CLIENT] req_cmd = 1 전송 (AprilTag 검출 요청)")

    start_time = time.monotonic()
    while True:

        ack_res = await client.read_holding_registers(address=RegAddr.READY_ACK, count=1, device_id=0)

        if not ack_res.isError():
            ack_val = ack_res.registers[0]
            if ack_val == AckStatus.READY:
                break
            if ack_val == AckStatus.ERROR:
                await cleanup_and_wait_idle(client)
                raise RuntimeError("서버 측 AprilTag 검출 실패 또는 범위 초과 (Ack=2)")

        if (time.monotonic() - start_time) > timeout_sec:
            # 타임아웃 발생 시에도 강제 초기화 시도
            await cleanup_and_wait_idle(client)
            raise TimeoutError(f"서버 응답 대기 시간 초과 ({timeout_sec}s)")

        await asyncio.sleep(CONFIG.poll_interval)

    # 2. Reg 10~16 일괄 읽기 (slave=0 명시)
    res = await client.read_holding_registers(address=DATA_START_ADDR, count=DATA_REG_COUNT, device_id=0)
    if res.isError():
        await cleanup_and_wait_idle(client)
        raise RuntimeError(f"레지스터 읽기 실패: {res}")

    detection = decode_detection(res.registers)

    print(
        f"[CLIENT] 수신 성공 -> Tag ID: {detection.tag_id} | "
        f"TCP 6-DoF(x,y,z,Rx,Ry,Rz): {tuple(round(v, 4) for v in detection.pose)}"
    )

    # 3. 수신 완료 후 초기화 및 대기
    await cleanup_and_wait_idle(client)

async def main() -> None:
    client = AsyncModbusTcpClient(host=CONFIG.target_ip, port=CONFIG.port)
    await client.connect()
    if not client.connected:
        print(f"서버 연결 실패 ({CONFIG.target_ip}:{CONFIG.port})")
        return

    try:
        print(f"서버 연결 성공 ({CONFIG.target_ip}:{CONFIG.port})")
        # 시작 전 서버가 초기화(IDLE) 상태인지 1회 보장
        await cleanup_and_wait_idle(client)

        for i in range(20):
            print(f"--- 테스트 요청 #{i + 1} ---")
            try:
                await request_tag_from_server(client)
            except (ModbusException, OSError, TimeoutError, RuntimeError) as err:
                print(f"[CLIENT 오류] {err}\n")
            await asyncio.sleep(1.0)
    finally:
        client.close()

def run() -> None:
    asyncio.run(main())

if __name__ == "__main__":
    run()
