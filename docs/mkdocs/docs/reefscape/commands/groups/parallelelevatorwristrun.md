# ParallelElevatorWristRun Command

The ParallelElevatorWristRun command group coordinates the simultaneous movement of both the elevator and wrist mechanisms to achieve faster overall positioning of the robot's manipulator system.

## Subsystem Requirements

This command group requires:
- [Elevator Subsystem](/5152_Reefscape/reefscape/subsystems/elevator)
- [Wrist Subsystem](/5152_Reefscape/reefscape/subsystems/wrist)

## Constructor Parameters

```java
public ParallelElevatorWristRun(
    ElevatorSubsystem elevatorSubsystem,
    WristSubsystem wristSubsystem,
    Distance elevatorHeight,
    Angle wristAngle)
```

- `elevatorSubsystem`: The elevator subsystem instance for vertical movement
- `wristSubsystem`: The wrist subsystem instance for angular movement
- `elevatorHeight`: Desired target height for the elevator using WPILib Distance units
- `wristAngle`: Desired target angle for the wrist using WPILib Angle units

## Configuration Requirements

1. Both subsystems must be properly configured with:
    - PID control parameters
    - Motion constraints
    - Soft limits
    - Zero offset calibration

2. The command uses proxy commands, so default commands should be configured for both subsystems

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/commands/groups/package-summary.html)
