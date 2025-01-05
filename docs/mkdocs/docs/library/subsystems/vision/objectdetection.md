# Object Detection Subsystem

The Object Detection subsystem is a vision processing system that uses PhotonVision to detect and track game objects on the field in real-time. It maintains lists of both stable (consistently tracked) and pending (newly detected) objects, providing filtered and reliable object detection data to other subsystems.

## Constructor

```java
public ObjectDetectionSubsystem(Supplier<Pose2d> robotPose, ObjectDetectionIO... io)
```

Parameters:
- `robotPose`: A supplier function that provides the current robot pose in field coordinates
- `io`: One or more ObjectDetectionIO interfaces representing the vision cameras

## Related Commands

The following commands utilize this subsystem:
- [DriveFacingBestObject](/5152_Template/library/commands/vision/drivefacingbestobject) - Automatically rotates the robot to face detected objects while driving
- [PathfindToBestObject](/5152_Template/library/commands/vision/pathfindtobestobject) - Autonomously navigates the robot to approach detected objects

## Configuration Requirements

The subsystem requires several constants to be configured in the ObjectDetectionConstants class:

1. Camera Configurations:
    - `CAMERA_CONFIGS`: Array of camera configuration parameters
    - `POSITION_MATCH_TOLERANCE`: Distance threshold for matching detected objects
    - `HISTORY_LENGTH`: Number of frames to keep in detection history
    - `REQUIRED_DETECTIONS`: Number of detections required for stability
    - `MISSING_FRAMES_THRESHOLD`: Number of missed frames before losing stability

2. Game Element Definitions:
    - `GAME_ELEMENTS`: Array mapping detected class IDs to game elements

3. PhotonVision Configuration:
    - Each camera must be properly configured in PhotonVision with:
        - Appropriate pipelines for object detection
        - Correct camera mounting position and angle
        - Calibrated camera parameters

## Requirements

- PhotonVision installed and configured on your robot
- At least one compatible camera connected and configured
- WPILib 2024 or newer

For detailed method documentation, refer to the [JavaDoc reference](PROJECT_ROOT/javadoc/frc/alotobots/library/subsystems/vision/photonvision/objectdetection/package-summary.html).
