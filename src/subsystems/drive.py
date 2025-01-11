from typing import NamedTuple, Tuple

from pathplannerlib.auto import DriveFeedforwards
from wpimath.controller import ProfiledPIDController
from wpimath.kinematics import (
    ChassisSpeeds,
    MecanumDriveWheelPositions,
    MecanumDriveWheelSpeeds,
)

from config import DriveMotorConfig
from commands2.subsystem import Subsystem
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkRelativeEncoder
from wpilib.drive import MecanumDrive
from wpilib import SmartDashboard
from wpimath.filter import SlewRateLimiter
from src.constants import Chassis


class Encoders(NamedTuple):
    front_left: SparkRelativeEncoder
    rear_left: SparkRelativeEncoder

    front_right: SparkRelativeEncoder
    rear_right: SparkRelativeEncoder


class Drive(Subsystem):
    __slots__ = (
        "drivetrain",
        "encoders",
        "forward_limiter",
        "sideways_limiter",
        "holonomic_controller",
        "kinematics",
    )
    """
    mecanum-based drive subsystem; controls the drivetrain
    """
    # TODO: add braking via PID controllers

    def __init__(
        self,
        config: DriveMotorConfig,
        motor_type: SparkLowLevel.MotorType = SparkLowLevel.MotorType.kBrushless,
    ):
        super().__init__()

        # initialize motors
        self.front_left = SparkMax(config.front_left_port, motor_type)
        self.rear_left = SparkMax(config.rear_left_port, motor_type)

        self.rear_right = SparkMax(config.rear_right_port, motor_type)
        self.front_right = SparkMax(config.front_right_port, motor_type)

        # apply configuration
        left_config = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        right_config = left_config.inverted(True)

        self.front_left.configure(
            left_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rear_left.configure(
            left_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.front_right.configure(
            right_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rear_right.configure(
            right_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.encoders = Encoders(
            self.front_left.getEncoder(),
            self.rear_left.getEncoder(),
            self.front_right.getEncoder(),
            self.rear_right.getEncoder(),
        )

        self.drivetrain = MecanumDrive(
            self.front_left, self.rear_left, self.rear_right, self.front_right
        )
        self.drivetrain.setExpiration(0.1)

        # soften the joystick inputs. it'll take 1 second to reach max speed.
        self.forward_limiter = SlewRateLimiter(1)
        self.sideways_limiter = SlewRateLimiter(1)

        self.pid = ProfiledPIDController(
            config.K_P, config.K_I, config.K_D, config.constraints
        )
        self.pid.setTolerance(0.1, 0)

        SmartDashboard.putData(self.drivetrain)

    def brake(self):
        """
        set drivetrain to brake mode.
        """
        left_brake_config = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        right_brake_config = left_brake_config.inverted(True)

        self.front_left.configure(
            left_brake_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rear_left.configure(
            left_brake_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.front_right.configure(
            right_brake_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rear_right.configure(
            right_brake_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def coast(self):
        """
        set drivetrain to coast mode.
        """
        left_coast_config = SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        right_coast_config = left_coast_config.inverted(True)

        self.front_left.configure(
            left_coast_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rear_left.configure(
            left_coast_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.front_right.configure(
            right_coast_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.rear_right.configure(
            right_coast_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

    def square_magnitude(self, input: float) -> float:
        """
        square a number's magnitude without changing sign. helper function for softening inputs
        """
        negativity = 1 if input > 0 else -1
        return (input**2) * negativity

    def drive(self, forward: float, sideways: float, rotate: float):
        """drive the robot using mecanum drive.

        Args:
            forward (float): the forward input
            sideways (float): the sideways input
            rotate (float): the rotation input
        """
        self.drivetrain.driveCartesian(  # try polar, perhaps?
            self.forward_limiter.calculate(forward),
            self.sideways_limiter.calculate(sideways),
            self.square_magnitude(rotate),
        )

    @classmethod
    def normalize_chassis_speeds(
        cls, speeds: ChassisSpeeds
    ) -> Tuple[float, float, float]:
        """normalize the chassis speeds to a max of 1.0

        Args:
            speeds (ChassisSpeeds): the speeds to normalize

        Returns:
            Tuple[float, float, float]: the normalized speeds
        """
        return (
            speeds.vx / Chassis.LINEAR_SPEED,
            speeds.vy / Chassis.LINEAR_SPEED,
            speeds.omega / Chassis.ANGULAR_SPEED,
        )

    def drive_relative(self, speeds: ChassisSpeeds, feedforward: DriveFeedforwards):
        """drive the robot based on chassisspeeds

        Args:
            speeds (ChassisSpeeds): the speeds to drive at
            feedforward (DriveFeedforwards): feedforwards to apply
        """
        # i have no clue how this feedforward thing works
        # TODO: add feedforward
        self.drive(*self.normalize_chassis_speeds(speeds))

    def get_wheel_positions(self) -> MecanumDriveWheelPositions:
        """gets the robot's current wheel positions from encoders.

        Returns:
            MecanumDriveWheelPositions: the current wheel position.
        """
        positions = MecanumDriveWheelPositions()
        positions.frontLeft = self.encoders.front_left.getPosition()
        positions.rearLeft = self.encoders.rear_left.getPosition()
        positions.frontRight = self.encoders.front_right.getPosition()
        positions.rearRight = self.encoders.rear_right.getPosition()
        return positions

    def get_wheel_speeds(self) -> MecanumDriveWheelSpeeds:
        """get's the robot's current wheel speeds from encoders.

        Returns:
            MecanumDriveWheelSpeeds: the current wheel speeds.
        """
        return MecanumDriveWheelSpeeds(
            self.encoders.front_left.getVelocity(),
            self.encoders.rear_left.getVelocity(),
            self.encoders.front_right.getVelocity(),
            self.encoders.rear_right.getVelocity(),
        )

    def stop(self):
        """stop the robot."""
        self.drivetrain.stopMotor()
