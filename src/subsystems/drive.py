from typing import NamedTuple
from config import MotorPorts
from commands2.subsystem import Subsystem
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkRelativeEncoder
from wpilib.drive import MecanumDrive
from wpilib.shuffleboard import Shuffleboard


class Drive(Subsystem):
    __slots__ = ("drivetrain", "encoders")
    """
    mecanum-based drive subsystem; controls the drivetrain
    """

    def __init__(
        self,
        motor_ports: MotorPorts,
        motor_type: SparkLowLevel.MotorType = SparkLowLevel.MotorType.kBrushless,
    ):
        super().__init__()

        # initialize motors
        front_left = SparkMax(motor_ports.front_left, motor_type)
        rear_left = SparkMax(motor_ports.rear_left, motor_type)

        rear_right = SparkMax(motor_ports.rear_right, motor_type)
        front_right = SparkMax(motor_ports.front_right, motor_type)

        # apply configuration
        left_config = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        right_config = left_config.inverted(True)

        front_left.configure(
            left_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        rear_left.configure(
            left_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        front_right.configure(
            right_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        rear_right.configure(
            right_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        class Encoders(NamedTuple):
            front_left: SparkRelativeEncoder
            rear_left: SparkRelativeEncoder

            front_right: SparkRelativeEncoder
            rear_right: SparkRelativeEncoder

        self.encoders = Encoders(
            front_left.getEncoder(),
            rear_left.getEncoder(),
            front_right.getEncoder(),
            rear_right.getEncoder(),
        )

        self.drivetrain = MecanumDrive(front_left, rear_left, rear_right, front_right)
        self.drivetrain.setExpiration(0.1)

        Shuffleboard.getTab("Comp").add(self.drivetrain)

    def drive(self, forward: float, sideways: float, rotate: float):
        self.drivetrain.driveCartesian(  # try polar, perhaps?
            forward, sideways, rotate
        )

    def stop(self):
        self.drivetrain.stopMotor()
