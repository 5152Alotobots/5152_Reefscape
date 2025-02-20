# WristOpenLoop Command

Command that runs on the wrist subsystem. Provides direct operator control of the wrist using percentage output.

## Required Subsystems
- [Wrist Subsystem](/5152_Reefscape/reefscape/subsystems/wrist)

## Constructor Parameters
```java
public WristOpenLoop(WristSubsystem wristSubsystem, DoubleSupplier input)
```
- `wristSubsystem`: The wrist subsystem instance to control
- `input`: A supplier for the percent output value (-1.0 to 1.0), typically from a joystick or controller

## Configuration Requirements
- None additional beyond wrist subsystem configuration
- Input values are automatically clamped to the wrist's MAX_OPEN_LOOP_PERCENTAGE limit

## Reference Documentation
- [DefaultWristOpenLoop Javadoc](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/wrist/commands/package-summary.html)
