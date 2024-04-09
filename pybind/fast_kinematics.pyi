import numpy as np
import torch

class FastKinematics:
    def __init__(self, urdf_path: str, num_of_robots: int, eef_name: str, verbose: bool = False) -> None:
        """
        :param urdf_path: path to the URDF file
        :param num_of_robots: number of robots in the URDF file
        :param eef_name: name of the end-effector
        :param verbose: whether to print debug information
        """

    def forward_kinematics(self, h_angs: np.ndarray, block_size: int = 256) -> np.ndarray:
        """
        forward kinematics in numpy

        :param h_angs: joint angles in 1d numpy array [q1, q2, q3, ...]
        :param block_size: block size for parallel computation
        :return: end-effector pose in 1d numpy array [x, y, z, qw, qx, qy, qz, ...]
        """

    def jacobian_mixed_frame(self, h_angs: np.ndarray, block_size: int = 256) -> np.ndarray:
        """
        mixed frame means the velocity is expressed in the end-effector frame
        but the angular velocity is expressed in the base frame

        :param h_angs: joint angles in 1d numpy array [q1, q2, q3, ...]
        :param block_size: block size for parallel computation
        :return: Jacobian matrix in 2d numpy array
        """

    def jacobian_world_frame(self, h_angs: np.ndarray, block_size: int = 256) -> np.ndarray:
        """
        both the velocity and angular velocity are expressed in the base frame

        :param h_angs: joint angles in 1d numpy array [q1, q2, q3, ...]
        :param block_size: block size for parallel computation
        :return: Jacobian matrix in 2d numpy array
        """

    def forward_kinematics_pytorch(self, t_angs: torch.Tensor, block_size: int = 256) -> torch.Tensor:
        """
        forward kinematics in PyTorch

        :param t_angs: joint angles in 1d torch tensor [q1, q2, q3, ...]
        :param block_size: block size for parallel computation
        :return: end-effector pose in 1d torch tensor [x, y, z, qw, qx, qy, qz, ...]
        """

    def jacobian_mixed_frame_pytorch(self, t_angs: torch.Tensor, block_size: int = 256) -> torch.Tensor:
        """
        mixed frame means the velocity is expressed in the end-effector frame
        but the angular velocity is expressed in the base frame

        :param t_angs: joint angles in 1d torch tensor [q1, q2, q3, ...]
        :param block_size: block size for parallel computation
        :return: Jacobian matrix in 2d torch tensor
        """

    def jacobian_world_frame_pytorch(self, t_angs: torch.Tensor, block_size: int = 256) -> torch.Tensor:
        """
        both the velocity and angular velocity are expressed in the base frame

        :param t_angs: joint angles in 1d torch tensor [q1, q2, q3, ...]
        :param block_size: block size for parallel computation
        :return: Jacobian matrix in 2d torch tensor
        """

    def get_num_of_active_joints(self) -> int:
        """
        get the number of joints that are not fixed in a single kinematic chain,
        """

    def get_num_of_joints(self) -> int:
        """
        get the total number of joints in a single kinematic chain,
        """

    def get_num_of_robots(self) -> int:
        """
        get the number of parallel robots
        """
