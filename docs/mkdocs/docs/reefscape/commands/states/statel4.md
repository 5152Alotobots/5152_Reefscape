# StateL4 Command

A command sequence that automates the process of placing game pieces at Level 4 scoring positions. This command coordinates multiple subsystems and uses a through-eject mechanism for more forceful piece placement at this maximum height.

## Subsystem Requirements

- [Elevator Subsystem](/5152_Reefscape/game/subsystems/elevator)
- [Wrist Subsystem](/5152_Reefscape/game/subsystems/wrist)
- [Coral Intake Subsystem](/5152_Reefscape/game/subsystems/coralintake)

## Constructor Parameters

```java
public StateL4(
    ElevatorSubsystem elevatorSubsystem,
    WristSubsystem wristSubsystem,
    CoralIntakeSubsystem coralIntakeSubsystem,
    Trigger coralIntakeReleaseTrigger)
```

- `elevatorSubsystem`: Controls vertical movement of the mechanism
- `wristSubsystem`: Controls angular position of the intake
- `coralIntakeSubsystem`: Controls the intake rollers
- `coralIntakeReleaseTrigger`: Button trigger that initiates piece release

## Configuration Requirements

1. All subsystems must have proper setpoints defined in their respective Constants files:
    - `ElevatorConstants.Setpoints.L4_PLACE`
    - `WristConstants.Setpoints.L4_PLACE`
    - `CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE`

2. Default commands should be configured for all subsystems since the command uses proxy commands

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/commands/states/package-summary.html)
