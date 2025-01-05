# AprilTag Subsystem

## Overview
The AprilTag subsystem is responsible for detecting and processing AprilTag fiducial markers using PhotonVision cameras. It provides robust pose estimation with statistical confidence measures by processing data from multiple cameras simultaneously.

## Features
- Multi-camera AprilTag detection and processing
- Statistical confidence calculations for pose estimates
- Automatic rejection of unreliable measurements
- Comprehensive logging of accepted and rejected poses
- Camera connection monitoring with alerts

## Constructor Parameters
The AprilTag subsystem requires two main parameters:

1. `AprilTagConsumer consumer`: A functional interface that receives processed AprilTag data, including:
    - Robot pose in field coordinates
    - Timestamp of the measurement
    - Statistical confidence measures (standard deviations)

2. `AprilTagIO... io`: Variable number of AprilTagIO interfaces, one for each camera

## Commands That Use This Subsystem
Currently there are no direct commands using this subsystem. The subsystem primarily acts as a data provider for other systems through its consumer interface.

## Required Configuration

1. Camera Configuration:
    - `CAMERA_CONFIGS` array must be defined in AprilTagConstants
    - Each camera must have proper configuration settings

2. Statistical Parameters:
    - `CAMERA_STD_DEV_FACTORS`: Must be >= 1.0 for each camera
    - `LINEAR_STD_DEV_BASE`: Base value for linear measurement standard deviation
    - `ANGULAR_STD_DEV_BASE`: Base value for angular measurement standard deviation

3. Field Layout:
    - `APRIL_TAG_LAYOUT`: Must contain the field's AprilTag layout configuration
    - Field dimensions must be properly configured

4. Processing Parameters:
    - `MAX_AMBIGUITY`: Maximum allowed pose ambiguity for single-tag detections
    - `MAX_Z_ERROR`: Maximum allowed Z-coordinate error

## Additional Information

The subsystem includes several safeguards for reliable operation:
- Automatic rejection of poses outside field boundaries
- Filtering of high-ambiguity single-tag detections
- Distance-based confidence scaling
- Camera disconnect detection and alerts

## Related Links
- [JavaDoc Reference](/5152_Template/javadoc/frc/alotobots/library/subsystems/vision/photonvision/apriltag/package-summary.html)
- [PhotonVision Documentation](https://docs.photonvision.org/en/latest/)
