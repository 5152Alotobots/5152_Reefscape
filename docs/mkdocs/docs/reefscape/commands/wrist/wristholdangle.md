# WristHoldAngle Command

The WristHoldAngle command is responsible for maintaining the wrist's current angular position by using velocity control set to zero. This creates a "holding" effect that keeps the wrist steady at its current angle.

## Subsystem Required

- [Wrist Subsystem](/5152_Reefscape/library/subsystems/wrist)

## Constructor Parameters

```java
public WristHoldAngle(WristSubsystem wristSubsystem)
```

- `wristSubsystem`: The wrist subsystem instance that this command will control

## Configuration Requirements

1. Velocity Control Configuration:
    - PID values must be properly configured for velocity control in the WristSubsystem
    - Feedforward values should be tuned to counteract gravity
    - Velocity measurement period and window size should be configured appropriately

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/wrist/commands/package-summary.html)
