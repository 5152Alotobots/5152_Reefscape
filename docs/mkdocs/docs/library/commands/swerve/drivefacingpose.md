# DriveFacingPose Command

## Overview
The DriveFacingPose command implements advanced pose-relative driving capabilities, automatically orienting the robot to face a target position while maintaining normal drive control. This command utilizes sophisticated geometry calculations and PID control for precise orientation management.

## Required Subsystems
- [SwerveDriveSubsystem](/5152_Template/library/subsystems/swerve/)

## Constructor Parameters
- `swerveDriveSubsystem`: The SwerveDrive subsystem instance this command will control
- `targetPose`: A supplier that provides the target Pose2d to face towards

## Configuration Requirements
The following must be configured in TunerConstants:
- PID values for angle control
- Maximum angular velocity and acceleration constraints
- Profile constraints for smooth angle transitions

## API Reference
For detailed API documentation, see the [DriveFacingPose Javadoc](/5152_Template/javadoc/frc/alotobots/library/commands/swervedrive/DriveFacingPose.html)
