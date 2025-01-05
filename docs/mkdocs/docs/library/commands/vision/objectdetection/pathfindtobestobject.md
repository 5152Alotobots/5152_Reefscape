# PathfindToBestObject Command

A command that automatically navigates the robot to approach detected game objects. This command handles full autonomous navigation to game elements, calculating appropriate approach positions while accounting for robot dimensions.

## Required Subsystems
- [Object Detection Subsystem](/5152_Template/library/subsystems/vision/objectdetection)
- [Swerve Drive Subsystem](/5152_Template/library/subsystems/swerve)

## Constructor Parameters

```java
public PathfindToBestObject(
    ObjectDetectionSubsystem objectDetectionSubsystem,
    SwerveDriveSubsystem swerveDriveSubsystem,
    GameElement... targetGameElementNames)
```

Parameters:
- `objectDetectionSubsystem`: The subsystem handling object detection
- `swerveDriveSubsystem`: The subsystem controlling robot movement
- `targetGameElementNames`: Array of game elements to target, in priority order. The robot will navigate to the highest-priority detected element.

## Configuration Requirements

The command requires proper configuration of:
1. Robot Physical Dimensions
    - Bumper length and width in Constants.tunerConstants
    - These dimensions are used to calculate safe approach distances

2. Path Planning Parameters
    - Any PathPlanner constraints required by the swerve drive subsystem
    - These affect how the robot navigates to the target position

## Behavior Details

- Identifies highest-priority detected game element
- Calculates optimal approach position based on:
    - Robot bumper dimensions
    - Approach angle to target
    - Game element position and orientation
- Generates and executes path to approach position
- Command times out after 0.1 seconds but generated path continues executing

For detailed method documentation, refer to the [JavaDoc reference](PROJECT_ROOT/javadoc/frc/alotobots/library/subsystems/vision/photonvision/objectdetection/commands/package-summary.html).
