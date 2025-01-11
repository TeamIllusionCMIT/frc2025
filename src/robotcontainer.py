from commands2 import RunCommand
from commands2.button import CommandXboxController
from photonlibpy.photonPoseEstimator import PoseStrategy
from wpilib import AnalogGyro, DriverStation, SmartDashboard
from wpimath.geometry import Rotation3d, Transform3d, Translation3d
from wpimath.trajectory import TrapezoidProfile

from src.subsystems.drive import Drive
from src.subsystems.odometry import Odometry
from src.subsystems.vision import Vision
from src.subsystems.arm import Arm

from config import DriveMotorConfig, PhotonCameraConfig, ArmConfig
from src.constants import unit, Chassis
from src.commands.driver_assist import DriverAssist

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from wpilib import RobotController


class RobotContainer:
    # slots to define what attributes the class should expect to hold
    __slots__ = (
        "vision",
        "drivetrain",
        "arm",
        "odometry",
        "controller",
        "auto_chooser",
        "driver_assist",
    )

    def __init__(self):
        self.controller = CommandXboxController(0)

        motor_ports = DriveMotorConfig(
            front_left_port=0,
            rear_left_port=1,
            front_right_port=2,
            rear_right_port=3,
            constraints=TrapezoidProfile.Constraints(
                Chassis.LINEAR_SPEED.magnitude, 4.1
            ),
        )  # 4.1 is from pathplanner's calculations
        self.drivetrain = Drive(motor_ports)
        self.arm = Arm(
            ArmConfig(4, 5, 6, 5.0, 0.0, 0.0, TrapezoidProfile.Constraints(1.0, 1.0))
        )

        # configure vision and odometry
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
        self.odometry = Odometry(self.drivetrain, self.vision, AnalogGyro(0))

        # configure pathplannerlib
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            self.odometry.estimate_pose,
            self.odometry.reset,
            lambda: self.odometry.kinematics.toChassisSpeeds(
                self.drivetrain.get_wheel_speeds()
            ),
            self.drivetrain.drive_relative,
            PPHolonomicDriveController(  # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0),  # Rotation PID constants
            ),
            config,
            self.should_flip_path,
            self.drivetrain,
        )

        self.auto_chooser = AutoBuilder.buildAutoChooser("barge park (5pts)")

        # add autonomous selection and battery voltage to dashboard
        SmartDashboard.putData(self.auto_chooser)
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage())

        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drivetrain.drive(
                    -self.controller.getLeftY(),
                    self.controller.getLeftX(),
                    self.controller.getRightX(),
                ),
                self.drivetrain,
            )
        )

        self.driver_assist = DriverAssist(self.vision, self.drivetrain, self.controller)

    def should_flip_path(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

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
        #
        # we don't have any buttons to configure bindings for yet, so we'll leave this empty

        self.controller.x().onTrue(
            RunCommand(self.arm.raise_arm, self.arm).andThen(
                RunCommand(lambda: self.arm.intake(1), self.arm)
            )
        )
        self.controller.leftTrigger(0.01).whileTrue(
            self.driver_assist.align_to_target()  # replace with alignment command
        )
        self.controller.rightTrigger(0.01).whileTrue(
            RunCommand(lambda: self.arm.expel(1), self.arm)
        )
        self.controller.b().onTrue(
            RunCommand(
                self.drivetrain.brake, self.drivetrain
            )  # brake when the button is pressed
        ).onFalse(
            RunCommand(
                self.drivetrain.coast, self.drivetrain
            )  # coast when the button is released
        )

    def getAutonomousCommand(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
