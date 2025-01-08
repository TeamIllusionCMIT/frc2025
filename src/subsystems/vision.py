from commands2.subsystem import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose3d, Translation3d
from src.constants import AprilTags, unit, Quantity
from config import PhotonCameraConfig
from typing import Tuple, Optional
from math import tan


class Vision(Subsystem):
    """
    apriltag vision via photonvision
    """

    __slots__ = ("camera", "pose_estimator")

    def __init__(self, config: PhotonCameraConfig):
        super().__init__()

        self.camera = PhotonCamera(config.camera_name)
        self.pose_estimator = PhotonPoseEstimator(
            fieldTags=AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
            strategy=config.pose_strategy,
            camera=self.camera,
            robotToCamera=config.robot_to_camera,
        )

    @property
    def driver_mode(self):
        """
        when driver mode is enabled, all computation is disabled and the camera is instead just streamed
        """
        self.camera.getDriverMode()

    @driver_mode.setter
    def driver_mode(self, enable: bool):
        self.camera.setDriverMode(enable)

    def latest_result(self) -> Optional[PhotonPipelineResult]:
        """get the latest photonvision result

        Returns:
            Optional[PhotonPipelineResult]: the result. can be none
        """
        if unread_results := self.camera.getAllUnreadResults():
            return unread_results[-1]  # most recent result

    def best_target(self) -> Optional[PhotonTrackedTarget]:
        """get the current best target

        Returns:
            Optional[PhotonTrackedTarget]: the target. can be none
        """
        if result := self.latest_result():
            return result.getBestTarget()

    def estimate_pose(self) -> Optional[Tuple[Optional[Pose3d], float]]:
        """estimate the current pose of the robot.

        Returns:
            Optional[Tuple[Optional[Pose3d], float]]: optional tuple of the pose and the timestamp. pose can be none
        """
        self.driver_mode = False

        if latest_result := self.latest_result():
            updated_pose = self.pose_estimator.update(latest_result)
            return (
                updated_pose.estimatedPose if updated_pose else None,
                latest_result.getTimestampSeconds(),
            )

    def get_apriltag_info(self, tag_id: int) -> Tuple[Quantity, Quantity]:
        """helper function to fetch physical properties of an apriltag.

        Args:
            tag_id (int): _description_

        Returns:
            Tuple[Quantity, Quantity]: _description_
        """
        return AprilTags.APRILTAG_MOUNT_HEIGHTS[
            tag_id
        ], 30 * unit.deg if tag_id in AprilTags.ANGLED_APRILTAGS else 0 * unit.deg  # type: ignore

    def calculate_distance(self, target: PhotonTrackedTarget) -> Translation3d:
        """calculate the distance between the robot and a target using trigonometry.

        Args:
            target (PhotonTrackedTarget): the target to calculate the distance to

        Returns:
            Translation3d: an object representing the translation from the robot to the target
        """
        # yaw is horizontal turn, pitch is vertical turn
        target_height, target_pitch = self.get_apriltag_info(target.getFiducialId())

        height_delta = PhotonCameraConfig.mount_height - target_height
        pitch_delta = (
            PhotonCameraConfig.mount_angle
            - (target.getPitch() * unit.deg)
            + target_pitch
        )

        distance = height_delta / tan(pitch_delta.to(unit.rad).magnitude)  # type: ignore

        # calculate left/right offset
        yaw_angle: Quantity = target.getYaw() * unit.deg
        yaw_angle_rad: float = yaw_angle.to(unit.rad).magnitude  # type: ignore

        lateral_offset = distance * tan(yaw_angle_rad)

        return Translation3d(
            distance, lateral_offset, target.getSkew()
        )  # this DEFINITELY needs to be checked
