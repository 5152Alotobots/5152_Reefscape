# Elevator Subsystem

The Elevator subsystem is responsible for controlling the vertical movement of the robot's elevator mechanism. It provides both closed-loop velocity/position control and open-loop manual control capabilities.

## Constructor and Parameters

```java
public ElevatorSubsystem(ElevatorIO io)
```

- `io`: Hardware interface for controlling the elevator mechanism
## Commands

The Elevator subsystem is used by the following commands:

- [DefaultElevatorRunAtVelocity](/5152_Reefscape/reefscape/commands/elevator/defaultelevatorrunatvelocity)
- [ElevatorOpenLoop](/5152_Reefscape/reefscape/commands/elevator/elevatoropenloop) - Command for manual control using percent output
- [ElevatorRunToHeight](/5152_Reefscape/reefscape/commands/elevator/elevatorruntoheight) - Moves the elevator to a specified target height and maintains it

## Configuration Requirements

1. PID Configuration:
    - Separate PID slots must be configured for each control type (VELOCITY, POSITION)
    - AT_SET_POINT_THRESHOLD must be set in ElevatorConstants

2. Hardware Limits:
    - MAX_HEIGHT and MIN_HEIGHT must be set in ElevatorConstants
    - MAX_OPEN_LOOP_PERCENTAGE must be configured for manual control limits

3. Sensor Configuration:
    - Position sensors must be properly configured and zeroed
    - Soft limits should be enabled to prevent mechanical damage

[View Javadoc Reference](5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/package-summary.html)
