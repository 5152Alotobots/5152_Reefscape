# LocalizationFusion Subsystem

The LocalizationFusion subsystem is a sophisticated pose estimation system that combines multiple sources of robot position tracking. It primarily uses Oculus Quest SLAM (Simultaneous Localization and Mapping) for continuous tracking, with AprilTag detection serving as both a backup and validation source.

## Construction

The LocalizationFusion subsystem requires several key components:

```java
public LocalizationFusion(
    PoseVisionConsumer poseConsumer,
    OculusPoseSource oculusSource,
    AprilTagPoseSource aprilTagSource,
    LoggedDashboardChooser<Command> autoChooser)
```

- `poseConsumer`: Interface that receives and processes pose updates
- `oculusSource`: Primary pose tracking using Oculus Quest SLAM
- `aprilTagSource`: Secondary pose tracking using AprilTags
- `autoChooser`: Autonomous mode selector for fallback pose setting

## Commands

The following commands interact with the LocalizationFusion subsystem:

- [RequestPositionResetViaTags](/5152_Reefscape/library/commands/vision/localizationfusion/requestpositionresetviatags): Requests a position reset using AprilTag detection

## Configuration

The subsystem behavior can be tuned through several constant groups in [LocalizationFusionConstants](/5152_Reefscape/library/subsystems/vision/localizationfusion/LocalizationFusionConstants.java):

### Validation Thresholds
- `APRILTAG_VALIDATION_THRESHOLD`: Maximum allowed difference between AprilTag and Quest poses during normal operation. Used to detect when the Quest tracking might be drifting and needs correction.
- `INIT_VALIDATION_THRESHOLD`: Stricter threshold used during system initialization to ensure accurate initial positioning. This tighter tolerance helps establish a reliable starting position.
- `DISABLED_RECALIBRATION_THRESHOLD`: Maximum allowed robot movement while disabled before triggering a recalibration. Helps detect if the robot has been moved significantly during disabled periods.
- `MAX_ROTATION_CHANGE_DEGREES`: Maximum allowed rotation change between consecutive pose updates. Helps filter out erroneous sudden rotations in pose estimation.

### Auto-Realignment
- `ENABLED`: Master switch for the auto-realignment feature. Allows completely disabling automatic pose corrections if needed.
- `THRESHOLD`: Amount of drift between Quest and AprilTag poses that triggers an auto-realignment. Smaller values mean more frequent corrections but potential interruptions.
- `MAX_MOVEMENT`: Maximum allowed robot movement during the stability check period. Ensures the robot is sufficiently still before attempting realignment.
- `STABILITY_TIME`: How long the robot must remain stable before attempting realignment. Prevents realignment attempts during robot motion.
- `COOLDOWN`: Minimum time required between auto-realignment attempts. Prevents excessive realignments from interrupting normal operation.

### Timing Parameters
- `POSE_UPDATE_INTERVAL`: Time between pose updates, matching the Quest's native 120Hz update rate. Controls how frequently the system processes new pose information.
- `QUEST_INIT_TIMEOUT`: Time allowed for Quest to establish stable tracking after startup. Ensures Quest has proper tracking before being used.
- `TAG_INIT_TIMEOUT`: Time allowed for AprilTag system to establish reliable detection. Ensures camera and detection system are working properly.
- `INITIAL_POSE_STABILITY_TIME`: Time required to validate the robot's initial position is stable. Prevents premature initialization if the robot is still being positioned.
- `RESET_TIMEOUT`: Maximum time allowed for a pose reset sequence to complete. Prevents the system from getting stuck in reset states.
- `MATCH_STARTUP_PERIOD_SECONDS`: Initial period after match start where special initialization rules apply. Allows for different behavior during the critical match start period.

### Initialization Requirements
- `MIN_QUEST_VALID_UPDATES`: Number of consistent pose updates required from Quest before considering it initialized. Ensures Quest tracking is stable before use.
- `MIN_TAG_VALID_UPDATES`: Number of consistent AprilTag detections required before considering the system initialized. Ensures reliable tag detection before use.
- `QUEST_INIT_GRACE_MULTIPLIER`: Factor that extends the initialization timeout if needed. Provides flexibility in initialization timing for challenging conditions.

## JavaDoc Reference
[Package Documentation](/5152_Reefscape/library/subsystems/vision/localizationfusion/package-summary.html)
