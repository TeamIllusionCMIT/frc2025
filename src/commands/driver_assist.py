from commands2 import RunCommand
from commands2.button import CommandXboxController
from src.subsystems.drive import Drive
from src.subsystems.vision import Vision


class DriverAssist:
    __slots__ = ("vision", "drivetrain", "controller", "__calculate_distance")
    """Holding class for driver assist commands."""

    def __init__(
        self, vision: Vision, drivetrain: Drive, controller: CommandXboxController
    ):
        self.vision = vision
        self.drivetrain = drivetrain
        self.controller = controller
        self.__calculate_distance = self.vision.calculate_distance  # function shortcut

    def align_to_target(self):
        """aligns the robot to a target using vision. still allows for some manual control"""
        target = self.vision.centermost_target()
        if not target:
            return RunCommand(lambda: self.drivetrain.drive(0, 0, 0), self.drivetrain)

        target_translation = self.__calculate_distance(target)
        forward_motion = (
            -self.controller.getLeftY()
            if abs(self.controller.getLeftY()) > 0.1
            else self.drivetrain.pid.calculate(target_translation.X(), 0)
        )
        side_motion = (
            self.controller.getLeftX()
            if abs(self.controller.getLeftX()) > 0.1
            else self.drivetrain.pid.calculate(target_translation.Y(), 0)
        )
        return RunCommand(
            lambda: self.drivetrain.drive(
                forward_motion,
                side_motion,
                self.drivetrain.pid.calculate(target.getSkew(), 0),
            ),
            self.drivetrain
        )
