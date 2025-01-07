from commands2 import ProfiledPIDSubsystem
from commands2.typing import TTrapezoidProfileState
from rev import SparkLowLevel, SparkMax
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from config import ArmConfig


class Arm(ProfiledPIDSubsystem):
    def __init__(self, config: ArmConfig):
        super().__init__(
            controller=ProfiledPIDController(
                config.K_P, config.K_I, config.K_D, constraints=config.constraints
            ),
            initial_position=0,
        )

        self.arm_motor = SparkMax(config.arm_port, SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.arm_motor.getEncoder()

        self.feedforward = ArmFeedforward  # set this later

    def getMeasurement(self) -> float:
        """
        gets the curent position of the arm.
        """
        return self.encoder.getPosition()

    def useOutput(self, output: float, setpoint: TTrapezoidProfileState):
        # we can add this in once we actually use wpilib sysid to profile the system
        # feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # in the mean time...
        feedforward = 0
        self.arm_motor.set(output + feedforward)
