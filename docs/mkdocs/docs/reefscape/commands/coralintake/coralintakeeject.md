# CoralIntakeEject Command

Runs the intake mechanism in reverse (negative speeds) to eject game pieces out the front of the intake. The command automatically stops when no game piece is detected.

## Required Subsystems
- [CoralIntake Subsystem](/5152_Reefscape/reefscape/subsystems/coralintake)

## Constructor Parameters
```java
public CoralIntakeEject(CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier input)
```
- `coralIntakeSubsystem`: The intake subsystem instance to control
- `input`: DoubleSupplier providing the eject speed (-MAX_OPEN_LOOP_EJECT_PERCENTAGE to 0.0) - negative values push out

## Configuration
No additional configuration required beyond subsystem configuration.

[JavaDoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/coralIntake/commands/package-summary.html)
