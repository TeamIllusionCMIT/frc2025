from math import pi, sqrt
from pint import UnitRegistry, Quantity
from dataclasses import dataclass, field

unit = UnitRegistry()


@dataclass(frozen=True)
class Vortex:
    FREE_SPEED: Quantity = 5676 * unit.rpm  # in RPM
    ENCODER_RESOLUTION: Quantity = (
        42 * unit.counts / unit.revolution
    )  # in counts per revolution


@dataclass(frozen=True)
class AprilTags:
    APRILTAG_MOUNT_HEIGHTS: dict[int, Quantity] = field(
        default_factory=lambda: {  # type: ignore
            1: (53.25 * unit.inch).to(unit.meter),
            2: (53.25 * unit.inch).to(unit.meter),
            3: (45.875 * unit.inch).to(unit.meter),
            4: (69 * unit.inch).to(unit.meter),
            5: (69 * unit.inch).to(unit.meter),
            6: (6.875 * unit.inch).to(unit.meter),
            7: (6.875 * unit.inch).to(unit.meter),
            8: (6.875 * unit.inch).to(unit.meter),
            9: (6.875 * unit.inch).to(unit.meter),
            10: (6.875 * unit.inch).to(unit.meter),
            11: (6.875 * unit.inch).to(unit.meter),
            12: (53.25 * unit.inch).to(unit.meter),
            13: (53.25 * unit.inch).to(unit.meter),
            14: (69 * unit.inch).to(unit.meter),
            15: (69 * unit.inch).to(unit.meter),
            16: (45.875 * unit.inch).to(unit.meter),
            17: (6.875 * unit.inch).to(unit.meter),
            18: (6.875 * unit.inch).to(unit.meter),
            19: (6.875 * unit.inch).to(unit.meter),
            20: (6.875 * unit.inch).to(unit.meter),
            21: (6.875 * unit.inch).to(unit.meter),
            22: (6.875 * unit.inch).to(unit.meter),
        }
    )
    ANGLED_APRILTAGS = (3, 5, 14, 15)  # these apriltags are angled upwards 30 degrees.
    APRILTAG_WIDTH: Quantity = (8.125 * unit.inch).to(unit.meter)  # in meters


@dataclass(frozen=True)
class Chassis:
    LENGTH = 0.82 * unit.meter
    WIDTH = 0.67 * unit.meter
    WHEEL_RADIUS: Quantity = (5 * unit.inch).to(unit.meter)  # type: ignore
    WHEEL_DIAMETER: Quantity = WHEEL_RADIUS * 2  # in meters
    WHEEL_CIRCUMFERENCE: Quantity = WHEEL_DIAMETER * pi * 2
    TRACK_WIDTH: Quantity = (5 * unit.inch).to(
        unit.meter
    )  # distance from left wheels to right wheels, in meters
    WHEEL_BASE: Quantity = (5 * unit.inch).to(
        unit.meter
    )  # distance from front wheels to back wheels, in meters
    GEAR_RATIO: float = 8.450  # gear ratio of the drivetrain. from the kitbot datasheet
    LINEAR_SPEED = ((WHEEL_RADIUS * Vortex.FREE_SPEED) / GEAR_RATIO).to(
        unit.mps
    )  # in m/s
    ROBOT_RADIUS: Quantity = (
        sqrt(((LENGTH.magnitude / 2) ** 2) + ((WIDTH.magnitude / 2) ** 2))
    ) * unit.meter  # in meters
    ANGULAR_SPEED = (LINEAR_SPEED / ROBOT_RADIUS).to(unit("rad/s"))  # in rad/s
