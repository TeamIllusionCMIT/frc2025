from commands2 import CommandScheduler, TimedCommandRobot
from wpilib import Preferences
from robotcontainer import RobotContainer


class Robot(TimedCommandRobot):
    __slots__ = ("auto_active", "core")

    def __init__(self):
        super().__init__()

        self.core = RobotContainer()

        Preferences.initBoolean("auto_active", False)

    def autonomousInit(self) -> None:
        self.auto_active = Preferences.getBoolean("auto_active", False)

    def autonomousPeriodic(self) -> None:
        ...

    def autonomousExit(self) -> None:
        self.core.drive.stop()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
