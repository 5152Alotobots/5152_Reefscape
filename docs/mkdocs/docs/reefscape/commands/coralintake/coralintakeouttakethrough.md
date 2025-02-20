# CoralIntakeOuttakeThrough Command

Runs the intake mechanism at positive speeds to eject game pieces by pulling through the intake mechanism in the intake direction. The command automatically stops when no game piece is detected. This is useful for ejecting game pieces through the intake side of the robot rather than the back side.

## Required Subsystems
- [CoralIntake Subsystem](/5152_Reefscape/reefscape/subsystems/coralintake)

## Constructor Parameters
```java
public CoralIntakeOuttakeThrough(CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier input)
```
- `coralIntakeSubsystem`: The intake subsystem instance to control
- `input`: DoubleSupplier providing the outtake speed (0.0 to MAX_OPEN_LOOP_OUTTAKE_PERCENTAGE) - positive values pull in

## Configuration
No additional configuration required beyond subsystem configuration.

[JavaDoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/coralIntake/commands/package-summary.html)
