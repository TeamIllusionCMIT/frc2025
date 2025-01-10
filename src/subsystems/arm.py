from commands2 import ProfiledPIDSubsystem
from commands2.typing import TTrapezoidProfileState
from rev import SparkLowLevel, SparkMax
from wpilib import MotorControllerGroup
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from config import ArmConfig
from src.constants import Vortex


class Arm(ProfiledPIDSubsystem):
    __slots__ = ("arm_motor", "roller", "encoder", "feedforward")

    def __init__(self, config: ArmConfig):
        super().__init__(
            controller=ProfiledPIDController(
                config.K_P, config.K_I, config.K_D, constraints=config.constraints
            ),
            initial_position=0,
        )
        self.arm_motor = SparkMax(config.arm_port, SparkLowLevel.MotorType.kBrushless)
        
        roller_motor_one = SparkMax(config.roller_port_one, SparkLowLevel.MotorType.kBrushless)
        roller_motor_two = SparkMax(config.roller_port_two, SparkLowLevel.MotorType.kBrushless)
        roller_motor_two.setInverted(True) # we're going to have to figure out which one to actually invert

        self.roller = MotorControllerGroup(roller_motor_one, roller_motor_two)

        self.encoder = self.arm_motor.getEncoder()

        self.feedforward = ArmFeedforward  # set this later

    def intake(self, speed: float):
        """
        runs the intake rollers at a specified speed to intake.
        """
        self.roller.set(speed)

    def expel(self, speed: float):
        """
        runs the intake rollers at a specified speed to expel.
        """
        self.roller.set(-speed)

    def stop_rollers(self):
        """
        stops the intake rollers.
        """
        self.roller.set(0)

    def move_arm(self, position: float):
        """
        sets the arm to a specified position in degrees.
        """
        self.setGoal(position)

    def getMeasurement(self) -> float:
        """
        gets the curent position of the arm in degrees. overrides the default method.
        """
        return (self.encoder.getPosition() / Vortex.ENCODER_RESOLUTION.magnitude) * 360

    def useOutput(self, output: float, setpoint: TTrapezoidProfileState):
        # we can add this in once we actually use wpilib sysid to profile the system
        # feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # in the mean time...
        feedforward = 0
        self.arm_motor.set(output + feedforward)
