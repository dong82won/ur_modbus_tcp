import asyncio
import time
from pymodbus.client import AsyncModbusTcpClient
from pymodbus.exceptions import ModbusException
from ur_modbus_tcp.shared.config import CONFIG
from ur_modbus_tcp.shared.register_map import (
    AckStatus,
    POSE_REG_COUNT,
    POSE_START_ADDR,
    RegAddr,
    decode_pose,
)

async def verify_heartbeat(client: AsyncModbusTcpClient, timeout_sec: float = 1.5) -> None:
    """하트비트 갱신 주기(~0.5s) 측정 및 서버 응답 없음(Timeout) 검증"""
    last_hb: int | None = None
    last_change_time = time.monotonic()

    while True:
        now = time.monotonic()
        try:
            res = await client.read_holding_registers(address=RegAddr.HEARTBEAT, count=1)

            if not res.isError():

                current_hb = res.registers[0]

                if last_hb is None:
                    last_hb = current_hb
                    last_change_time = now
                    print(f"[HB 시작] 초기값: {current_hb}")

                elif current_hb != last_hb:
                    dt = now - last_change_time
                    print(f"[HB 갱신] {last_hb} -> {current_hb} (경과 시간: {dt:.3f}s)")
                    last_hb = current_hb
                    last_change_time = now

        except (ModbusException, OSError):
            # 서버 강제 종료로 소켓이 끊어진 경우에도 타임아웃 판정까지 루프 유지
            pass

        # 1.5초(0.5초 주기의 3배) 이상 값이 변하지 않으면 서버 다운으로 판정
        if (now - last_change_time) > timeout_sec:
            print(f"\n[경고] 하트비트 정지 감지! ({timeout_sec}s 초과 - PC 서버 중단 또는 네트워크 단절)")
            break

        await asyncio.sleep(0.1)


async def request_pose_from_server(client: AsyncModbusTcpClient, timeout_sec: float = 2.0) -> None:
    """서버에 6-DoF 좌표를 요청하고 수신 후 req_cmd를 초기화"""

    # 1. 서버에 데이터 요청 트리거 (req_cmd = 1)
    await client.write_register(address=RegAddr.REQ_CMD, value=1)
    print("[CLIENT] req_cmd = 1 전송 (6-DoF 좌표 요청)")

    # 2. 서버의 ready_ack != 0 대기 (1=성공, 2=에러)
    start_time = time.monotonic()

    while True:

        ack_res = await client.read_holding_registers(address=RegAddr.READY_ACK, count=1)

        if not ack_res.isError():
            ack_val = ack_res.registers[0]

            if ack_val == AckStatus.READY:
                break
            if ack_val == AckStatus.ERROR:
                await client.write_register(address=RegAddr.REQ_CMD, value=0)
                raise RuntimeError("서버(PC) 측 6-DoF 좌표 산출/인코딩 오류 (Ack=2)")

        if (time.monotonic() - start_time) > timeout_sec:
            raise TimeoutError(f"서버 응답(READY_ACK) 대기 시간 초과 ({timeout_sec}s)")

        await asyncio.sleep(CONFIG.poll_interval)

    # 3. 10~15번 레지스터 일괄 읽기 및 실수 복원
    pose_res = await client.read_holding_registers( address=POSE_START_ADDR, count=POSE_REG_COUNT)

    if pose_res.isError():
        raise RuntimeError(f"레지스터 읽기 실패: {pose_res}")

    pose_6dof = decode_pose(pose_res.registers)
    print(f"[CLIENT] 수신된 6-DoF (x, y, z, r, p, y): {pose_6dof}")

    # 4. 수신 완료 후 req_cmd = 0 복귀 (다음 요청 준비)
    await client.write_register(address=RegAddr.REQ_CMD, value=0)
    print("[CLIENT] req_cmd = 0 초기화 완료\n")


async def main() -> None:
    client = AsyncModbusTcpClient(host=CONFIG.target_ip, port=CONFIG.port)
    await client.connect()

    if not client.connected:
        print(f"서버 연결 실패 ({CONFIG.target_ip}:{CONFIG.port})")
        return

    hb_task: asyncio.Task[None] | None = None

    try:
        print(f"서버 연결 성공 ({CONFIG.target_ip}:{CONFIG.port})")

        # 백그라운드 하트비트 감시 시작
        hb_task = asyncio.create_task(verify_heartbeat(client, timeout_sec=1.5))

        for i in range(3):
            if hb_task.done():
                break

            print(f"--- 테스트 요청 #{i + 1} ---")
            try:
                await request_pose_from_server(client)
            except (ModbusException, OSError, TimeoutError, RuntimeError) as err:
                print(f"[CLIENT 오류] 요청 중단: {err}")
                break

            await asyncio.sleep(1.0)

        if not hb_task.done():
            print(
                "[CLIENT] 6-DoF 3회 테스트 완료. 하트비트 감시 유지 중 "
                "(서버 터미널에서 Ctrl+C로 종료해 타임아웃을 검증하세요)..."
            )
            await hb_task

    finally:
        if hb_task and not hb_task.done():
            hb_task.cancel()

        client.close()
        print("[CLIENT] 연결 종료")


def run() -> None:
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    run()
