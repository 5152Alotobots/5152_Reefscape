# ElevatorHoldHeight Command

A command that maintains the elevator's position at its current height using closed-loop control. This command is useful for maintaining a specific height during game piece manipulation or when precise positioning is required.

## Required Subsystems
- [Elevator Subsystem](/5152_Reefscape/library/subsystems/elevator)

## Constructor Parameters
```java
public ElevatorHoldHeight(ElevatorSubsystem elevatorSubsystem)
```
- `elevatorSubsystem`: The elevator subsystem to control

## Configuration
- Ensure PID constants are properly tuned for position holding in ElevatorConstants
- AT_SET_POINT_THRESHOLD must be configured in ElevatorConstants
- The command will use the appropriate PID slot based on the current game element
- Brake mode should be properly configured in the motor controller

[View Javadoc Reference](5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/commands/package-summary.html)
