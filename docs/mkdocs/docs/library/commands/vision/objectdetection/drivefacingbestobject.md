# DriveFacingBestObject Command

A command that automatically rotates the robot to face detected game objects while allowing manual translation control. The command enables semi-automated gameplay by handling rotation while letting drivers control forward/backward and sideways movement.

## Required Subsystems
- [Object Detection Subsystem](/5152_Template/library/subsystems/vision/objectdetection)
- [Swerve Drive Subsystem](/5152_Template/library/subsystems/swerve)

## Constructor Parameters

```java
public DriveFacingBestObject(
    ObjectDetectionSubsystem objectDetectionSubsystem,
    SwerveDriveSubsystem swerveDriveSubsystem,
    GameElement... targetGameElementNames)
```

Parameters:
- `objectDetectionSubsystem`: The subsystem handling object detection
- `swerveDriveSubsystem`: The subsystem controlling robot movement
- `targetGameElementNames`: Array of game elements to target, in priority order. The robot will face the highest-priority detected element.

## Behavior Details

- Automatically rotates to face the highest-priority detected game element
- Allows full manual control of translation (forward/backward/sideways)
- Falls back to standard manual drive when no objects are detected
- Includes brief manual rotation override capability with 0.1s timeout
- Integrates with both [DriveFacingPose](/5152_Template/library/commands/swerve/drivefacingpose) and [DefaultDrive](/5152_Template/library/commands/swerve/defaultdrive) commands

For detailed method documentation, refer to the [JavaDoc reference](PROJECT_ROOT/javadoc/frc/alotobots/library/subsystems/vision/photonvision/objectdetection/commands/package-summary.html).
