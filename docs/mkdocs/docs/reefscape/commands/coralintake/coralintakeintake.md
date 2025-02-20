# CoralIntakeIntake Command

Runs the intake mechanism at positive speeds to collect game pieces by pulling them inward. The command automatically stops when a game piece is detected by the intake sensor.

## Required Subsystems
- [CoralIntake Subsystem](/5152_Reefscape/reefscape/subsystems/coralintake)

## Constructor Parameters
```java
public CoralIntakeIntake(CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier input)
```
- `coralIntakeSubsystem`: The intake subsystem instance to control
- `input`: DoubleSupplier providing the intake speed (0.0 to MAX_OPEN_LOOP_INTAKE_PERCENTAGE) - positive values pull in

## Configuration
No additional configuration required beyond subsystem configuration.

[JavaDoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/coralIntake/commands/package-summary.html)
