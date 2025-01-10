from commands2.subsystem import Subsystem
from wpimath.estimator import MecanumDrivePoseEstimator
from wpimath.kinematics import MecanumDriveKinematics
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from src.constants import Chassis
from src.subsystems.drive import Drive
from src.subsystems.vision import Vision
from wpilib import AnalogGyro, Field2d, Timer
from wpilib import SmartDashboard


class Odometry(Subsystem):
    __slots__ = (
        "drivetrain",
        "vision",
        "gyro",
        "kinematics",
        "pose_estimator",
        "vision_timer",
        "field",
    )

    def __init__(self, drivetrain: Drive, vision: Vision, gyro: AnalogGyro):
        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.gyro = gyro

        track_width_m = Chassis.TRACK_WIDTH.magnitude
        wheel_base_m = Chassis.WHEEL_BASE.magnitude

        self.kinematics = MecanumDriveKinematics(
            # positive means:                 right              front
            frontLeftWheel=Translation2d(-track_width_m / 2, wheel_base_m / 2),
            rearLeftWheel=Translation2d(-track_width_m / 2, -wheel_base_m / 2),
            frontRightWheel=Translation2d(track_width_m / 2, wheel_base_m / 2),
            rearRightWheel=Translation2d(track_width_m / 2, -wheel_base_m / 2),
        )

        self.pose_estimator = MecanumDrivePoseEstimator(
            kinematics=self.kinematics,
            gyroAngle=Rotation2d.fromDegrees(0),
            wheelPositions=self.drivetrain.get_wheel_positions(),
            initialPose=Pose2d(
                Translation2d(0, 0), Rotation2d.fromDegrees(0)
            ),  # TODO: make it so u can actually pick a starting point in the dashboard
        )

        self.vision_timer = Timer()
        self.field = Field2d()
        SmartDashboard.putData(self.field)

    def periodic(self) -> None:
        """update the pose estimator with the current wheel positions. additionally, add vision data every one second."""
        # update pose (and field)
        new_pose = self.pose_estimator.update(
            self.gyro.getRotation2d(), self.drivetrain.get_wheel_positions()
        )
        self.field.setRobotPose(new_pose)

        # every 1 second, add vision measurement
        if self.vision_timer.hasElapsed(1):
            print(new_pose)
            self.vision_timer.reset()
            self.add_vision_measurement()

    def add_vision_measurement(self) -> None:
        """add photonvision's pose estimation to the pose estimator."""
        vision_result = self.vision.estimate_pose()
        if vision_result:
            vision_estimate, timestamp = vision_result
            if vision_estimate:
                self.pose_estimator.addVisionMeasurement(
                    vision_estimate.toPose2d(), timestamp
                )

    def estimate_pose(self) -> Pose2d:
        """get the current estimated pose from the pose estimator.

        Returns:
            Pose2d: the robot's current pose estimate
        """
        return self.pose_estimator.getEstimatedPosition()

    def reset(self, pose: Pose2d) -> None:
        """reset the odometry to a certain position.

        Args:
            pose (Pose2d): the new pose to reset to
        """
        self.pose_estimator.resetPose(pose)
