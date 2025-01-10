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


class DriveMotorConfig(NamedTuple):
    front_left_port: int
    rear_left_port: int

    front_right_port: int
    rear_right_port: int


class ArmConfig(NamedTuple):
    arm_port: int
    roller_port_one: int
    roller_port_two: int

    K_P: float
    K_I: float
    K_D: float

    constraints: TrapezoidProfile.Constraints
