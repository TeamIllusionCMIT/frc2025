from commands2.subsystem import Subsystem
from wpimath.estimator import MecanumDrivePoseEstimator
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelPositions
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from constants import Chassis
from src.subsystems.drive import Drive
from src.subsystems.vision import Vision
from wpilib.shuffleboard import Shuffleboard


class Odometry(Subsystem):
    def __init__(self, drive: Drive, vision: Vision):
        super().__init__()
        self.drive = drive
        self.vision = vision

        track_width_m = Chassis.TRACK_WIDTH.magnitude
        wheel_base_m = Chassis.WHEEL_BASE.magnitude

        self.kinematics = MecanumDriveKinematics(
            # positive means...                  right                      front
            frontLeftWheel=Translation2d(-track_width_m / 2, wheel_base_m / 2),
            rearLeftWheel=Translation2d(-track_width_m / 2, -wheel_base_m / 2),
            frontRightWheel=Translation2d(track_width_m / 2, wheel_base_m / 2),
            rearRightWheel=Translation2d(track_width_m / 2, -wheel_base_m / 2),
        )

        self.pose_estimator = MecanumDrivePoseEstimator(
            kinematics=self.kinematics,
            gyroAngle=Rotation2d.fromDegrees(0),
            wheelPositions=MecanumDriveWheelPositions(),
            initialPose=Pose2d(
                Translation2d(0, 0), Rotation2d.fromDegrees(0)
            ),  # TODO: make it so u can actually pick a starting point in the dashboard
        )

        Shuffleboard.getTab("Comp")

    def periodic(self) -> None:
        vision_result = self.vision.estimate_pose()
        if not vision_result:
            return None

        vision_estimate, timestamp = vision_result

        if not vision_estimate:
            return None

        self.pose_estimator.addVisionMeasurement(vision_estimate.toPose2d(), timestamp)

    def estimate_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()
