import os
from contextlib import contextmanager
import logging
import threading
import time
from typing import Any, Protocol, cast
import cv2
import numpy as np
from numpy.typing import NDArray
from pupil_apriltags import Detector
import pyrealsense2 as _rs
from ur_modbus_tcp.hand_in_eye_coor import HandEyeTransformer
from ur_modbus_tcp.shared.config import CONFIG
from ur_modbus_tcp.shared.register_map import TagDetection

# C++ 확장 모듈(pyrealsense2)에 대한 Pylance 속성 인식 오류 방지
rs: Any = _rs
logger = logging.getLogger(__name__)


@contextmanager
def suppress_c_output():
    """C 라이브러리(pupil_apriltags)에서 발생하는 stdout/stderr 터미널 강제 출력을 무시"""
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stdout = os.dup(1)
    old_stderr = os.dup(2)
    try:
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(old_stdout, 1)
        os.dup2(old_stderr, 2)
        os.close(old_stdout)
        os.close(old_stderr)
        os.close(devnull)


class _AprilTagDetection(Protocol):
    """pupil_apriltags.Detection의 불완전한 내부 타입 힌트 보정용 프로토콜"""

    tag_id: int
    decision_margin: float
    center: NDArray[np.float64]
    corners: NDArray[np.float64]
    pose_R: NDArray[np.float64] | None
    pose_t: NDArray[np.float64] | None


class RealSenseAprilTagProvider:
    """Intel RealSense 연속 프레임 수집, tag36h11 검출, 핸드아이 변환 및 시각화 제공자"""

    def __init__(
        self,
        transformer: HandEyeTransformer | None = None,
        allowed_tag_ids: set[int] | None = None,
        reconnect_interval: float = 2.0,
    ) -> None:
        self.transformer = transformer or HandEyeTransformer()
        self.allowed_tag_ids = allowed_tag_ids
        self._reconnect_interval = reconnect_interval

        self._frame_lock = threading.Lock()
        self._detector_lock = threading.Lock()

        self._latest_gray: NDArray[np.uint8] | None = None
        self._latest_preview: NDArray[np.uint8] | None = None
        self._camera_params: tuple[float, float, float, float] | None = None
        self._camera_matrix: NDArray[np.float64] | None = None
        self._dist_coeffs = np.zeros((4, 1), dtype=np.float64)

        self._is_connected = False
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._pipeline = rs.pipeline()

        self._detector = Detector(
            families=CONFIG.tag_family,
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self._connect_camera()

        self._running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True
        )
        self._capture_thread.start()

    def _connect_camera(self) -> bool:
        """RealSense 장치 연결 확인 및 파이프라인 시작"""
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                raise RuntimeError("연결된 Intel RealSense 장치가 없습니다.")

            dev = devices[0]
            dev_name = dev.get_info(rs.camera_info.name)
            dev_serial = dev.get_info(rs.camera_info.serial_number)

            rs_config = rs.config()
            rs_config.enable_stream(
                rs.stream.color,
                CONFIG.cam_width,
                CONFIG.cam_height,
                rs.format.bgr8,
                CONFIG.cam_fps,
            )
            profile = self._pipeline.start(rs_config)

            color_stream = profile.get_stream(
                rs.stream.color
            ).as_video_stream_profile()
            intr = color_stream.get_intrinsics()
            fx, fy, cx, cy = (
                float(intr.fx),
                float(intr.fy),
                float(intr.ppx),
                float(intr.ppy),
            )

            self._camera_params = (fx, fy, cx, cy)
            self._camera_matrix = np.array(
                [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            self._is_connected = True

            logger.info(
                f"RealSense 연결 성공 [{dev_name} (S/N: {dev_serial})] "
                f"({CONFIG.cam_width}x{CONFIG.cam_height}@{CONFIG.cam_fps}fps) | "
                f"Intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
            )
            return True

        except Exception as err:
            self._disconnect_cleanup()
            logger.error(
                f"RealSense 카메라 초기화 실패 (USB 연결/권한/해상도 확인 필요): {err}"
            )
            return False

    def _disconnect_cleanup(self) -> None:
        """연결 해제 또는 오류 시 상태 초기화 및 잔상 프레임 폐기"""
        self._is_connected = False
        self._camera_params = None
        self._camera_matrix = None
        with self._frame_lock:
            self._latest_gray = None
            self._latest_preview = None
        try:
            self._pipeline.stop()
        except Exception:
            pass

    def _run_detector(
        self, gray_img: NDArray[np.uint8]
    ) -> list[_AprilTagDetection]:
        """스레드 안전한 AprilTag 검출 및 유효성 필터링"""
        if self._camera_params is None:
            return []

        with self._detector_lock:
            # C레벨 알고리즘 경고(Error, more than one new minima found) 터미널 출력 무시
            with suppress_c_output():
                raw_detections = cast(
                    list[_AprilTagDetection],
                    self._detector.detect(
                        gray_img,
                        estimate_tag_pose=True,
                        camera_params=self._camera_params,
                        tag_size=CONFIG.tag_size_m,
                    ),
                )

        return [
            d
            for d in raw_detections
            if d.decision_margin >= CONFIG.min_decision_margin
            and d.pose_R is not None
            and d.pose_t is not None
            and (
                self.allowed_tag_ids is None
                or int(d.tag_id) in self.allowed_tag_ids
            )
        ]

    def _draw_detections(
        self,
        bgr_img: NDArray[np.uint8],
        detections: list[_AprilTagDetection],
    ) -> NDArray[np.uint8]:
        """검출된 마커에 바운딩 박스, 3D 좌표축, X/Y/Z 거리 텍스트 오버레이"""
        vis = bgr_img.copy()
        img_h, img_w = vis.shape[:2]

        if not detections:
            cv2.putText(
                vis,
                "Searching AprilTag (tag36h11)...",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 200, 255),
                2,
            )
            return vis

        # 가장 신뢰도가 높은 태그 강조 표시
        best = max(detections, key=lambda item: float(item.decision_margin))

        for d in detections:
            if d.pose_R is None or d.pose_t is None:
                continue

            is_primary = d is best
            box_color = (0, 255, 0) if is_primary else (255, 180, 0)

            # 1. 마커 4개 꼭짓점 바운딩 박스 그리기
            pts = d.corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(
                vis, [pts], isClosed=True, color=box_color, thickness=2
            )

            # 중심점 표시
            cx_px, cy_px = int(round(d.center[0])), int(round(d.center[1]))
            cv2.circle(vis, (cx_px, cy_px), 4, (0, 0, 255), -1)

            # 2. 3D X, Y, Z 축 그리기 (빨강=X, 초록=Y, 파랑=Z)
            if self._camera_matrix is not None:
                rvec, _ = cv2.Rodrigues(d.pose_R)
                cv2.drawFrameAxes(
                    vis,
                    self._camera_matrix,
                    self._dist_coeffs,
                    rvec,
                    d.pose_t,
                    CONFIG.tag_size_m * 0.6,
                )

            cam_x, cam_y, cam_z = (float(v) for v in d.pose_t.reshape(3))
            tcp_x, tcp_y, tcp_z, _, _, _ = self.transformer.cam_to_tcp_pose(
                d.pose_R, d.pose_t
            )

            lines = [
                f"ID: {int(d.tag_id)} (margin: {d.decision_margin:.1f})",
                f"Cam XYZ:({cam_x:+.3f}, {cam_y:+.3f}, {cam_z:+.3f}) m",
                f"TCP XYZ:({tcp_x:+.3f}, {tcp_y:+.3f}, {tcp_z:+.3f}) m", # Base -> TCP로 표기 변경
            ]

            # 텍스트가 화면 밖으로 잘리지 않도록 위치 보정
            min_x = max(10, min(img_w - 330, int(np.min(d.corners[:, 0]))))
            min_y = int(np.min(d.corners[:, 1]))
            text_y_start = min_y - 62 if min_y > 75 else int(np.max(d.corners[:, 1])) + 22
            text_y_start = max(20, min(img_h - 65, text_y_start))

            # 가독성을 위한 반투명/어두운 배경 박스
            cv2.rectangle(
                vis,
                (min_x - 4, text_y_start - 16),
                (min_x + 330, text_y_start + 46),
                (20, 20, 20),
                -1,
            )
            for idx, text in enumerate(lines):
                cv2.putText(
                    vis,
                    text,
                    (min_x, text_y_start + idx * 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    box_color if idx == 0 else (255, 255, 255),
                    1 if idx > 0 else 2,
                    cv2.LINE_AA,
                )

        return vis

    def _capture_loop(self) -> None:
        while self._running:
            if not self._is_connected:
                time.sleep(self._reconnect_interval)
                if self._running:
                    self._connect_camera()
                continue

            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                bgr_img = np.asanyarray(color_frame.get_data())
                gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

                preview_img: NDArray[np.uint8] | None = None
                if CONFIG.show_preview:
                    detections = self._run_detector(gray_img)
                    preview_img = self._draw_detections(bgr_img, detections)

                with self._frame_lock:
                    self._latest_gray = gray_img
                    self._latest_preview = preview_img

            except Exception as err:
                if self._running:
                    logger.error(f"RealSense 프레임 수신 중단 (카메라 분리/오류): {err}")
                    self._disconnect_cleanup()

    def get_preview_frame(self) -> NDArray[np.uint8]:
        """메인 GUI 스레드에서 cv2.imshow()로 출력할 최신 프레임 반환"""
        with self._frame_lock:
            if self._latest_preview is not None:
                return self._latest_preview.copy()

        # 카메라 미연결 또는 대기 중일 때 안내 화면 반환
        blank = np.zeros((CONFIG.cam_height, CONFIG.cam_width, 3), dtype=np.uint8)
        cv2.putText(
            blank,
            "Waiting for Intel RealSense camera...",
            (40, CONFIG.cam_height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 100, 255),
            2,
        )
        return blank

    # def detect_and_transform(self) -> TagDetection:
    #     """UR 요청 시 최신 프레임에서 tag36h11을 검출하고 Robot Base 6-DoF 반환"""
    #     if not self._is_connected or self._camera_params is None:
    #         raise RuntimeError("RealSense 카메라가 연결되어 있지 않습니다.")

    #     with self._frame_lock:
    #         if self._latest_gray is None:
    #             raise RuntimeError("카메라 프레임이 아직 수신되지 않았습니다.")
    #         gray_img = self._latest_gray.copy()

    #     valid_detections = self._run_detector(gray_img)
    #     if not valid_detections:
    #         raise RuntimeError("유효한 tag36h11 마커가 검출되지 않았습니다.")

    #     best = max(valid_detections, key=lambda d: float(d.decision_margin))
    #     if best.pose_R is None or best.pose_t is None:
    #         raise RuntimeError("AprilTag 포즈 행렬(pose_R/pose_t) 추정 실패")

    #     # 카메라 기준 포즈 -> 로봇 TCP 기준 6-DoF 변환
    #     tcp_pose = self.transformer.cam_to_tcp_pose(best.pose_R, best.pose_t)
    #     return TagDetection(tag_id=int(best.tag_id), pose=tcp_pose)

    def detect_and_transform(self, max_retries: int = 6) -> TagDetection:
        """UR 요청 시 최신 프레임에서 tag36h11을 검출. 실패 시 최대 6회(약 0.3초) 재시도"""
        if not self._is_connected or self._camera_params is None:
            raise RuntimeError("RealSense 카메라가 연결되어 있지 않습니다.")

        for attempt in range(max_retries):
            with self._frame_lock:
                if self._latest_gray is None:
                    gray_img = None
                else:
                    gray_img = self._latest_gray.copy()

            if gray_img is not None:
                valid_detections = self._run_detector(gray_img)

                # 유효한 태그를 찾은 경우 즉시 좌표 반환
                if valid_detections:
                    best = max(valid_detections, key=lambda d: float(d.decision_margin))
                    if best.pose_R is not None and best.pose_t is not None:
                        tcp_pose = self.transformer.cam_to_tcp_pose(best.pose_R, best.pose_t)
                        return TagDetection(tag_id=int(best.tag_id), pose=tcp_pose)

            # 실패 시 다음 프레임이 캡처될 때까지 잠시 대기 (약 30fps 기준 50ms)
            time.sleep(0.05)

        # 지정된 횟수만큼 재시도했음에도 못 찾은 경우에만 에러(2) 발생
        raise RuntimeError(f"유효한 tag36h11 마커가 검출되지 않았습니다. ({max_retries}회 재시도 실패)")

    def stop(self) -> None:
        """서버 종료 시 RealSense 파이프라인 안전 해제"""
        self._running = False
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.5)
        self._disconnect_cleanup()
        logger.info("RealSense 파이프라인이 정상 종료되었습니다.")
