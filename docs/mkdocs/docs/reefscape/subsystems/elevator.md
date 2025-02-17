# Elevator Subsystem

The Elevator subsystem is responsible for controlling the vertical movement of the robot's elevator mechanism. It provides both closed-loop position control and open-loop manual control capabilities, with different PID configurations based on the game element being handled.

## Constructor and Parameters

```java
public ElevatorSubsystem(ElevatorIO io, Supplier<GameElement> elementInIntake)
```

- `io`: Hardware interface for controlling the elevator mechanism
- `elementInIntake`: Supplier that provides information about the current game element in the intake

## Commands

The Elevator subsystem is used by the following commands:

- [DefaultElevatorOpenLoop](/5152_Reefscape/library/commands/elevator/defaultelevatoropenloop) - Default command for manual control using percent output
- [ElevatorHoldHeight](/5152_Reefscape/library/commands/elevator/elevatorholdheight) - Maintains the elevator at its current height
- [ElevatorRunToHeight](/5152_Reefscape/library/commands/elevator/elevatorruntoheight) - Moves the elevator to a specified target height

## Configuration Requirements

1. PID Configuration:
    - Separate PID slots must be configured for each game element type (NONE, CORAL, ALGAE)
    - AT_SET_POINT_THRESHOLD must be set in ElevatorConstants

2. Hardware Limits:
    - MAX_HEIGHT and MIN_HEIGHT must be set in ElevatorConstants
    - MAX_OPEN_LOOP_PERCENTAGE must be configured for manual control limits

3. Sensor Configuration:
    - Position sensors must be properly configured and zeroed
    - Soft limits should be enabled to prevent mechanical damage

[View Javadoc Reference](5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/package-summary.html)
