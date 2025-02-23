# ElevatorOpenLoop Command

A command that provides manual control of the elevator using percent output control. Allows direct operator control through a joystick or other input device.

## Required Subsystems
- [Elevator Subsystem](/5152_Reefscape/reefscape/subsystems/elevator)

## Constructor Parameters
```java
public ElevatorOpenLoop(ElevatorSubsystem elevatorSubsystem, DoubleSupplier percent)
```
- `elevatorSubsystem`: The elevator subsystem to control
- `percent`: Supplier for the control input (-1.0 to 1.0), typically from a joystick or controller

## Configuration
- Ensure MAX_OPEN_LOOP_PERCENTAGE is properly set in ElevatorConstants
- Configure joystick deadband if needed for smoother control
- The command will automatically clamp inputs to the configured MAX_OPEN_LOOP_PERCENTAGE

[View Javadoc Reference](5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/commands/package-summary.html)
