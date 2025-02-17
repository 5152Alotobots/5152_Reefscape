# ElevatorRunToHeight Command

A command that moves the elevator to a specified target height using closed-loop control. This command completes once the elevator reaches the target height within the specified tolerance.

## Required Subsystems
- [Elevator Subsystem](/5152_Reefscape/library/subsystems/elevator)

## Constructor Parameters
```java
public ElevatorRunToHeight(ElevatorSubsystem elevatorSubsystem, Distance targetHeight)
```
- `elevatorSubsystem`: The elevator subsystem to control
- `targetHeight`: The desired height for the elevator to reach (automatically clamped between MIN_HEIGHT and MAX_HEIGHT)

## Configuration
- Ensure PID constants are properly tuned for each game element type
- Configure MAX_HEIGHT and MIN_HEIGHT in ElevatorConstants
- AT_SET_POINT_THRESHOLD must be set appropriately for position tolerance
- Soft limits should be enabled to prevent mechanical damage
- The command will use the appropriate PID slot based on the current game element

[View Javadoc Reference](5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/commands/package-summary.html)