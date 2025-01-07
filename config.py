from typing import NamedTuple
from wpimath.geometry import Transform3d
from wpimath.trajectory import TrapezoidProfile
from photonlibpy.photonPoseEstimator import PoseStrategy
from pint import Quantity


class PhotonCameraConfig(NamedTuple):
    camera_name: str
    pose_strategy: PoseStrategy
    robot_to_camera: Transform3d

    mount_height: Quantity  # meters
    mount_angle: Quantity  # degrees


class MotorPorts(NamedTuple):
    front_left: int
    rear_left: int

    front_right: int
    rear_right: int


class ArmConfig(NamedTuple):
    arm_port: int
    top_roller_port: int
    bottom_roller_port: int

    K_P: float
    K_I: float
    K_D: float

    constraints: TrapezoidProfile.Constraints
