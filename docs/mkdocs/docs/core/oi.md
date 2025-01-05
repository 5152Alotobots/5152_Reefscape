# Operator Interface (OI)

The Operator Interface class manages all driver control inputs and button bindings for the robot. It provides a centralized location for configuring and accessing driver controls.

## Purpose
The OI class handles:
- Driver controller configuration
- Joystick axis mapping
- Button binding definitions
- Speed control modifiers

## Configuration Required
1. Controller port assignments (default port 0 for driver controller)
2. Deadband value for joystick inputs
3. Axis mapping for drive controls
4. Button binding assignments

## Dependencies
- WPILib CommandXboxController for input handling
- Command bindings referenced in [RobotContainer](/5152_Template/core/robotcontainer)

## JavaDoc Reference
Complete documentation can be found [here](PROJECT_ROOT/javadoc/frc/alotobots/package-summary.html)
