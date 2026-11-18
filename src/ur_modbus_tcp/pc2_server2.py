import os
# Ubuntu Wayland 환경에서 OpenCV(Qt) 폰트 경고 메시지 도배 방지
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.*=false"

import asyncio
import logging
import threading
import time
import cv2
from collections.abc import Callable
from pymodbus.server import ModbusTcpServer
from pymodbus.simulator import DataType, SimData, SimDevice

from ur_modbus_tcp.realsense_pose2 import RealSenseAprilTagProvider

from ur_modbus_tcp.shared.config import CONFIG
from ur_modbus_tcp.shared.register_map import (
    AckStatus,
    DATA_START_ADDR,
    RegAddr,
    TagDetection,
    encode_detection,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


def create_sim_device() -> SimDevice:
    """pymodbus 최신 규격에 맞춘 시뮬레이션 디바이스 메모리 생성"""
    registers = SimData(
        address=0,
        values=[0] * CONFIG.register_count,
        datatype=DataType.REGISTERS,
    )
    return SimDevice(id=0, simdata=[registers])


async def handshake_worker(
    server: ModbusTcpServer,
    detect_fn: Callable[[], TagDetection],
) -> None:
    """Modbus 클라이언트(UR)의 요청을 감시하고 검출 결과를 전달하는 비동기 워커"""
    device_id = 0
    func_code = 3
    was_requested = False
    heartbeat = 0
    loop_count = 0

    while True:
        try:
            # 1. 주기적 하트비트 갱신
            loop_count += 1
            if loop_count >= int(0.5 / CONFIG.poll_interval):
                loop_count = 0
                heartbeat = (heartbeat + 1) % 65536
                await server.async_setValues(
                    device_id, func_code, RegAddr.HEARTBEAT, [heartbeat]
                )

            # 2. UR 요청 확인
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
                    result = await asyncio.to_thread(detect_fn)
                    regs = encode_detection(result)

                    await server.async_setValues(
                        device_id, func_code, DATA_START_ADDR, regs
                    )
                    await server.async_setValues(
                        device_id, func_code, RegAddr.READY_ACK, [AckStatus.READY]
                    )
                    logger.info(
                        f"전송 완료 | Tag ID: {result.tag_id} | TCP 6-DoF: "
                        f"{tuple(round(v, 4) for v in result.pose)}"
                    )

                except Exception as det_err:
                    logger.warning(f"검출/변환 실패 (UR에 ERROR=2 전달): {det_err}")
                    await server.async_setValues(
                        device_id, func_code, RegAddr.READY_ACK, [AckStatus.ERROR]
                    )

            elif req_cmd == 0 and was_requested:
                was_requested = False
                await server.async_setValues(
                    device_id, func_code, RegAddr.READY_ACK, [AckStatus.IDLE]
                )

        except asyncio.CancelledError:
            break
        except Exception as loop_err:
            logger.exception(f"통신 루프 예기치 않은 오류: {loop_err}")

        await asyncio.sleep(CONFIG.poll_interval)


async def async_server_main(
    provider: RealSenseAprilTagProvider, stop_event: threading.Event
) -> None:
    """ModbusTcpServer 인스턴스화 및 실행을 담당하는 비동기 메인 함수"""
    device = create_sim_device()

    # 이벤트 루프가 완전히 활성화된 상태에서 서버 객체 생성
    server = ModbusTcpServer(context=device, address=(CONFIG.server_host, CONFIG.port))

    # 🌟 [핵심 수정] 서버가 네트워크 Listen을 시작할 때까지 대기
    await server.serve_forever(background=True)
    logger.info(f"Ubuntu Modbus TCP 서버 대기 중 ({CONFIG.server_host}:{CONFIG.port})...")

    # 서버 준비가 완료된 후 핸드셰이크 워커 실행
    hs_task = asyncio.create_task(handshake_worker(server, provider.detect_and_transform))


    # 메인 스레드에서 종료 신호(stop_event)를 보낼 때까지 대기
    while not stop_event.is_set():
        await asyncio.sleep(0.2)

    # 안전 종료 절차
    logger.info("Modbus 서버 종료 절차 시작...")
    hs_task.cancel()
    await server.shutdown()


def run_async_server(
    provider: RealSenseAprilTagProvider, stop_event: threading.Event
) -> None:
    """백그라운드 스레드 진입점"""
    asyncio.run(async_server_main(provider, stop_event))


def run() -> None:
    provider = RealSenseAprilTagProvider()
    stop_event = threading.Event()

    # 1. Modbus 통신을 백그라운드 스레드로 분리
    server_thread = threading.Thread(
        target=run_async_server, args=(provider, stop_event), daemon=True
    )
    server_thread.start()

    # 2. 메인 스레드에서 OpenCV GUI 루프 실행
    window_name = "RealSense AprilTag (tag36h11) - Press 'q' to quit"

    try:
        if CONFIG.show_preview:
            while True:
                frame = provider.get_preview_frame()
                cv2.imshow(window_name, frame)

                key = cv2.waitKey(33) & 0xFF
                if key in (ord("q"), 27):
                    logger.info("사용자 입력(q/ESC)으로 메인 루프를 종료합니다.")
                    break
        else:
            while True:
                time.sleep(1.0)

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt(Ctrl+C) 감지됨.")
    finally:
        stop_event.set()
        server_thread.join(timeout=3.0)
        cv2.destroyAllWindows()
        provider.stop()
        logger.info("프로그램이 정상 종료되었습니다.")


if __name__ == "__main__":
    run()
