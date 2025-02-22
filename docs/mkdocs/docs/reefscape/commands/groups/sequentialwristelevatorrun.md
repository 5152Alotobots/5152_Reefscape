# SequentialWristElevatorRun Command

The SequentialWristElevatorRun command group coordinates the movement of both the wrist and elevator mechanisms in a specific order, moving the wrist to position first, followed by the elevator movement. This sequential operation can be useful for avoiding interference or when specific movement patterns are required.

## Subsystem Requirements

This command group requires:
- [Wrist Subsystem](/5152_Reefscape/game/subsystems/wrist)
- [Elevator Subsystem](/5152_Reefscape/game/subsystems/elevator)

## Constructor Parameters

```java
public SequentialWristElevatorRun(
    WristSubsystem wristSubsystem,
    ElevatorSubsystem elevatorSubsystem,
    Distance elevatorHeight,
    Angle wristAngle)
```

- `wristSubsystem`: The wrist subsystem instance for angular movement
- `elevatorSubsystem`: The elevator subsystem instance for vertical movement
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