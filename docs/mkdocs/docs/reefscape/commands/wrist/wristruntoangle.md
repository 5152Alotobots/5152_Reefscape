# WristRunToAngle Command

Command that moves the wrist to a specific angle using closed-loop position control. Used for automatic positioning of the wrist for various game piece manipulation tasks.

## Required Subsystems
- [Wrist Subsystem](/5152_Reefscape/reefscape/subsystems/wrist)

## Constructor Parameters
```java
public WristRunToAngle(WristSubsystem wristSubsystem, Angle angle)
```
- `wristSubsystem`: The wrist subsystem instance to control
- `angle`: The target angle to move to, using WPILib's Units system (e.g., Degrees.of(90))

## Configuration Requirements
1. PID Configuration
    - Position mode PID gains must be properly tuned in WristTalonFXRealConstants
    - AT_TARGET_ANGLE_POSITION_THRESHOLD must be set to an appropriate value
    - AT_TARGET_ANGLE_TIME_THRESHOLD must be set to an appropriate value

2. Motion Parameters
    - Input angle is automatically clamped between MIN_ANGLE and MAX_ANGLE
    - Proper gravity compensation values should be configured

## Reference Documentation
- [WristRunToAngle Javadoc](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/wrist/commands/package-summary.html)
