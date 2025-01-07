# frc2025

this project uses [rye](https://rye.astral.sh) to manage dependencies

# a quick intro to pint
pint is a units library used in this project to allow for easy conversion and to ensure that units are always stated. it's most importantly used in the constant.py and config.py files, which is where you describe physical properties.

a quick summary:
```py
from constants import unit

>>> length = 5 * unit.inch
<Quantity(5, 'inch')>

>>> length.to(unit.cm)
<Quantity(12.7, 'centimeter')>

# these are NOT regular numbers; to get the regular numbers you can use magnitude:
>>> length.to(unit.cm).magnitude
12.7
```

# file overview
## constants.py
variables that never change. variables and dataclasses in this file describe real-world things, such as apriltag heights/rotations and motor specifications. uses pint.

## config.py
classes to configure different things, namely the vision subsystem and motors.

## robot.py
the main file run. really just imports from robotcontainer.py

## robotcontainer.py
the bulk of the robot code. initializes the subsystems

## src
### subsystems
each of the primary components of the robot.

- arm
- drive (mecanum drivetrain)
- vision
- odometry (uses pose estimation alongside vision)