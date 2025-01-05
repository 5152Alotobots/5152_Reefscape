# Reset Pose Command

A command that resets the Oculus headset's position tracking to align with a specified pose on the field.

## Required Subsystems
- [Oculus Navigation Subsystem](/5152_Template/library/subsystems/vision/questnav)

## Constructor Parameters
```java
public ResetPoseCommand(OculusSubsystem oculus, Pose2d targetPose)
```
- `oculus`: The Oculus subsystem instance to reset
- `targetPose`: The target pose to reset the tracking to

## Configuration
The target pose should be specified in field coordinates.

## Reference Documentation

[JavaDoc Reference](/5152_Template/javadoc/frc/alotobots/library/subsystems/vision/questnav/commands/ResetPoseCommand.html)
