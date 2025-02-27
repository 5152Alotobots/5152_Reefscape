# ElevatorHoldHeight Command

The ElevatorHoldHeight command maintains the elevator's current height position using velocity control set to zero. This command is specifically designed for use in autonomous mode, as the default command provides similar functionality in teleop mode.

## Subsystem Required

- [Elevator Subsystem](/5152_Reefscape/library/subsystems/elevator)

## Constructor Parameters

```java
public ElevatorHoldHeight(ElevatorSubsystem elevatorSubsystem)
```

- `elevatorSubsystem`: The elevator subsystem instance that this command will control

## Configuration Requirements

1. Velocity Control Configuration:
    - PID values must be properly configured for velocity control in the ElevatorSubsystem
    - Feedforward values should be tuned to counteract gravity
    - Velocity measurement period and window size should be configured appropriately

2. Usage Notes:
    - This command should only be used in autonomous routines
    - For teleop operation, use the default command instead

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/commands/package-summary.html)
