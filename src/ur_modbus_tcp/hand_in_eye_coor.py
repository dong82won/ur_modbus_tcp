import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from ur_modbus_tcp.shared.register_map import Pose6DoF

# TODO: Eye-in-Hand 캘리브레이션 결과 (4x4 동차변환행렬: Robot TCP -> Camera)
# OpenCV calibrateHandEye()로 얻은 R_tcp2cam, t_tcp2cam을 조립한 행렬입니다.
DEFAULT_T_TCP_CAM: NDArray[np.float64] = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.05],  # 예: 카메라가 로봇 TCP에서 Y방향으로 5cm 떨어짐
        [0.0, 0.0, -1.0, 0.10],  # 예: Z방향으로 10cm 앞으로 나와 있음
        [0.0, 0.0, 0.0, 1.0]
    ],
    dtype=np.float64,
)


class HandEyeTransformer:
    """카메라 좌표계 기준 AprilTag 포즈를 로봇 손목(TCP) 좌표계로 변환 (Eye-in-Hand)"""

    def __init__(self, t_tcp_cam: NDArray[np.float64] = DEFAULT_T_TCP_CAM) -> None:
        if t_tcp_cam.shape != (4, 4):
            raise ValueError(f"4x4 행렬이 필요하지만 {t_tcp_cam.shape}가 입력되었습니다.")
        self.t_tcp_cam = t_tcp_cam.astype(np.float64)

    @staticmethod
    def build_homogeneous_matrix(
        rot_mat: NDArray[np.float64], trans_vec: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        t_mat = np.eye(4, dtype=np.float64)
        t_mat[:3, :3] = rot_mat
        t_mat[:3, 3] = trans_vec.reshape(3)
        return t_mat

    def cam_to_tcp_pose(
        self, cam_r_tag: NDArray[np.float64], cam_t_tag: NDArray[np.float64]
    ) -> Pose6DoF:
        """
        Eye-in-Hand 변환: T_tcp_tag = T_tcp_cam @ T_cam_tag
        반환: Robot TCP 기준 (x, y, z [m], roll, pitch, yaw [rad])
        """
        t_cam_tag = self.build_homogeneous_matrix(cam_r_tag, cam_t_tag)
        t_tcp_tag = self.t_tcp_cam @ t_cam_tag

        x, y, z = (float(v) for v in t_tcp_tag[:3, 3])

        # URScript의 rpy2rotvec([r, p, y])와 일치하는 Extrinsic 'XYZ' 오일러 각 추출
        roll, pitch, yaw = (
            float(a)
            for a in Rotation.from_matrix(t_tcp_tag[:3, :3]).as_euler("XYZ", degrees=False)
        )
        return (x, y, z, roll, pitch, yaw)
