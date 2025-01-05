# SwerveDrive Subsystem

## Overview
The SwerveDrive subsystem is a sophisticated drive system that enables omnidirectional movement of the robot. It manages four swerve modules (each containing drive and turn motors), handles odometry calculations, and provides autonomous path-following capabilities.

## Commands that use the subsystem
- [DefaultDrive Command](/5152_Template/library/commands/swerve/defaultdrive): Provides standard teleoperated control
- [DriveFacingAngle Command](/5152_Template/library/commands/swerve/drivefacingangle): Maintains a specific robot heading while driving
- [DriveFacingPose Command](/5152_Template/library/commands/swerve/drivefacingpose): Orients the robot to face a target position while driving
- [Characterization Commands](/5152_Template/library/commands/swerve/characterization): Used for system identification and calibration

## Configuration Requirements

### Hardware Requirements
1. Four Swerve Drive Modules (Front Left, Front Right, Back Left, Back Right)
    - Each module requires:
        - Drive motor with encoder
        - Turn motor with absolute encoder
2. Gyroscope (NavX, Pigeon, or similar IMU)

### Configuration in TunerConstants
All configuration is handled through TunerConstants, including:
- Module configurations (gear ratios, conversions, PID values)
- PathPlanner settings
- Holonomic drive controller settings
- Module positions and dimensions
- Maximum velocity and acceleration limits
- Drive characterization values (kS, kV, kA)
- Wheel radius parameters
- Absolute encoder offsets

See the [JavaDoc Reference](/5152_Template/javadoc/frc/alotobots/library/subsystems/swervedrive/package-summary.html) for detailed configuration options.

### Additional Notes
- Odometry updates run on a separate high-frequency thread
- Vision measurements can be incorporated for pose estimation
- Supports both teleop and autonomous operation
- PathPlanner integration for autonomous path following
- Built-in system identification capabilities for tuning
