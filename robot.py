from commands2 import CommandScheduler, TimedCommandRobot
from wpilib import Preferences, CameraServer
from src.robotcontainer import RobotContainer


class Robot(TimedCommandRobot):
    __slots__ = ("auto_active", "core")

    def __init__(self):
        super().__init__()

        # so we can see the webcam feed
        CameraServer().launch()

        self.core = RobotContainer()

        Preferences.initBoolean("auto_active", False)

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def autonomousInit(self) -> None:
        self.auto_active = Preferences.getBoolean("auto_active", False)

    def autonomousPeriodic(self) -> None: ...

    def autonomousExit(self) -> None:
        self.core.drivetrain.stop()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
