# WristRunToAngle Command

A command that moves the wrist to a specified target angle using closed-loop position control. The command can either end once the target is reached or continuously maintain the position.

## Required Subsystems
- [Wrist Subsystem](/5152_Reefscape/reefscape/subsystems/wrist)

## Constructor Parameters
```java
public WristRunToAngle(WristSubsystem wristSubsystem, Angle angle, boolean holdPosition)
```
- `wristSubsystem`: The wrist subsystem instance to control
- `angle`: The target angle to move to (automatically clamped between MIN_ANGLE and MAX_ANGLE)
- `holdPosition`: Optional parameter to maintain position after reaching target

## Configuration Requirements
1. PID Configuration
   - Position mode PID gains must be properly tuned
   - AT_SET_POINT_THRESHOLD must be set to an appropriate value

2. Hardware Limits
   - MAX_ANGLE and MIN_ANGLE must be configured in WristConstants
   - Soft limits should be enabled to prevent mechanical damage

3. Motor Configuration
   - Brake mode should be enabled for position holding
   - Feedback sensor must be properly configured and zeroed
   - Proper gravity compensation values should be configured

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/wrist/commands/package-summary.html)
