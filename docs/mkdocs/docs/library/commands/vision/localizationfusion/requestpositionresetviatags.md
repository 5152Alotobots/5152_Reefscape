# RequestPositionResetViaTags Command

A command that initiates a position reset of the robot's pose estimation using AprilTag detection. This command attempts to align the Oculus Quest SLAM tracking with the observed AprilTag positions.

## Required Subsystems
- [LocalizationFusion](/5152_Reefscape/library/subsystems/vision/localizationfusion): The localization subsystem that manages pose estimation

## Constructor Parameters

```java
public RequestPositionResetViaTags(LocalizationFusion localizationFusion)
```

- `localizationFusion`: The LocalizationFusion subsystem instance that will perform the reset

## Configuration

This command uses the validation thresholds configured in the LocalizationFusion subsystem:

- `APRILTAG_VALIDATION_THRESHOLD`: Maximum allowed difference between AprilTag and Quest poses
- `RESET_TIMEOUT`: Maximum time allowed for reset sequence completion

No additional configuration is required for this command.

## JavaDoc Reference
[Package Documentation](/5152_Reefscape/library/subsystems/vision/localizationfusion/commands/package-summary.html)
