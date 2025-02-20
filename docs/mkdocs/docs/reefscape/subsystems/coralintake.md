# Coral Intake Subsystem

The Coral Intake subsystem manages the robot's game piece collection and ejection mechanism. It uses a SparkFlex motor controller and a CANrange sensor to control and monitor the intake mechanism.

## Constructor and Parameters

```java
public CoralIntakeSubsystem(CoralIntakeIO io)
```

- `io`: Hardware interface for controlling the intake mechanism (either CoralIntakeIOVortexReal for hardware or simulation implementation)

## Commands

The following commands use this subsystem:
- [CoralIntakeIntake](/5152_Reefscape/reefscape/commands/coralintake/coralintakeintake) - Runs intake until a game piece is detected
- [CoralIntakeOuttake](/5152_Reefscape/reefscape/commands/coralintake/coralintakeouttake) - Ejects game piece in reverse until no piece is detected
- [CoralIntakeOuttakeThrough](/5152_Reefscape/reefscape/commands/coralintake/coralintakeouttakethrough) - Ejects game piece forward until no piece is detected

## Configuration Requirements

1. CAN IDs in Constants file:
    - INTAKE_MOTOR_CAN_ID
    - INTAKE_CANRANGE_ID

2. PID Constants in CoralIntakeVortexRealConstants:
    - KP, KI, KD, KF for velocity control

3. Motor Safety Limits:
    - STATOR_AMP_LIMIT
    - TORQUE_FORWARD_AMP_LIMIT
    - TORQUE_REVERSE_AMP_LIMIT

4. Operation Limits in CoralIntakeConstants:
    - MAX_OPEN_LOOP_PERCENTAGE
    - MAX_OPEN_LOOP_INTAKE_PERCENTAGE
    - MAX_OPEN_LOOP_OUTTAKE_PERCENTAGE

[JavaDoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/coralIntake/package-summary.html)
