# StateCoralStation Command

A command sequence that automates the process of intaking game pieces from the coral station loading zone. This command coordinates the movement of multiple subsystems to perform the intake operation safely and efficiently.

## Subsystem Requirements

- [Elevator Subsystem](/5152_Reefscape/game/subsystems/elevator)
- [Wrist Subsystem](/5152_Reefscape/game/subsystems/wrist)
- [Coral Intake Subsystem](/5152_Reefscape/game/subsystems/coralintake)

## Constructor Parameters

```java
public StateCoralStation(
    ElevatorSubsystem elevatorSubsystem,
    WristSubsystem wristSubsystem,
    CoralIntakeSubsystem coralIntakeSubsystem)
```

- `elevatorSubsystem`: Controls vertical movement of the mechanism
- `wristSubsystem`: Controls angular position of the intake
- `coralIntakeSubsystem`: Controls the intake rollers

## Configuration Requirements

1. All subsystems must have proper setpoints defined in their respective Constants files:
    - `ElevatorConstants.Setpoints.CORAL_STATION`
    - `WristConstants.Setpoints.CORAL_STATION`
    - `CoralIntakeConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE`

2. Default commands should be configured for all subsystems since the command uses proxy commands

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/commands/states/package-summary.html)
