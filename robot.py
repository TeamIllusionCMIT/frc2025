from commands2 import CommandScheduler, TimedCommandRobot, WaitCommand
from wpilib import Preferences, CameraServer, SmartDashboard
from src.robotcontainer import RobotContainer


class Robot(TimedCommandRobot):
    __slots__ = ("auto_active", "core")

    def __init__(self):
        super().__init__()

        # so we can see the webcam feed
        CameraServer().launch()

        self.core = RobotContainer()

        Preferences.initBoolean("auto_active", False)

    def disabledPeriodic(self) -> None:
        # TODO: edit this to allow for some slight tolerance in the starting position
        SmartDashboard.putBoolean(
            "position correct?",
            self.core.getAutonomousCommand()._startingPose
            == self.core.odometry.estimate_pose(),
        )

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def autonomousInit(self) -> None:
        self.core.drivetrain.brake()
        self.auto_active = Preferences.getBoolean("auto_active", False)

    def autonomousPeriodic(self) -> None: ...

    def autonomousExit(self) -> None:
        self.core.drivetrain.stop()
        WaitCommand(0.5).schedule()
        self.core.drivetrain.coast()

    def teleopInit(self) -> None:
        self.core.drivetrain.coast()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
