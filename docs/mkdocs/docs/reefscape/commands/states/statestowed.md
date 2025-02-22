# StateStowed Command

A command sequence that safely returns the robot's mechanisms to their stowed/home positions. This command ensures proper sequencing of movements to avoid collisions, moving the wrist first followed by the elevator.

## Subsystem Requirements

- [Elevator Subsystem](/5152_Reefscape/game/subsystems/elevator)
- [Wrist Subsystem](/5152_Reefscape/game/subsystems/wrist)

## Constructor Parameters

```java
public StateStowed(
    ElevatorSubsystem elevatorSubsystem,
    WristSubsystem wristSubsystem)
```

- `elevatorSubsystem`: Controls vertical movement of the mechanism
- `wristSubsystem`: Controls angular position of the intake

## Configuration Requirements

1. All subsystems must have proper setpoints defined in their respective Constants files:
    - `ElevatorConstants.Setpoints.STOWED`
    - `WristConstants.Setpoints.STOWED`

2. Default commands should be configured for all subsystems since the command uses proxy commands

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/commands/states/package-summary.html)