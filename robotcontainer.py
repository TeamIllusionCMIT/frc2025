from commands2 import RunCommand
from commands2.button import CommandXboxController
from photonlibpy.photonPoseEstimator import PoseStrategy
from wpimath.geometry import Rotation3d, Transform3d, Translation3d

from src.subsystems.drive import Drive
from src.subsystems.odometry import Odometry
from src.subsystems.vision import Vision
from config import MotorPorts, PhotonCameraConfig
from constants import unit


class RobotContainer:
    # slots to define what attributes the class should expect to hold
    __slots__ = ("vision", "drive", "arm", "odometry", "controller")

    def __init__(self):
        self.controller = CommandXboxController(0)

        motor_ports = MotorPorts(0, 1, 2, 3)
        self.drive = Drive(motor_ports)
        self.arm = None

        vision_config = PhotonCameraConfig(
            "apriltags",
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Transform3d(
                Translation3d.fromFeet(0, 0, 0), Rotation3d(0, 0, 0)
            ),  # transformation from center of robot. rotation is probably accurate here
            0 * unit.cm,
            0 * unit.deg,
        )
        self.vision = Vision(vision_config)
        self.odometry = Odometry(self.drive, self.vision)

        self.drive.setDefaultCommand(
            RunCommand(
                lambda: self.drive.drive(
                    -self.controller.getLeftY(),
                    self.controller.getLeftX(),
                    self.controller.getRightX(),
                ),
                self.drive,
            )
        )

    def configureBindings(self):
        # configure button bindings; here's some examples
        #
        # for buttons:
        # self.controller.a().onTrue(
        #   InstantCommand(lambda: ...)
        # )
        #
        # for triggers:
        # self.controller.leftTrigger(0.01).onTrue(
        #     InstantCommand(lambda: ...)
        # )
        ...

    def getAutonomousCommand(self):
        # dummy function to allow selection of autonomous based on position
        ...
