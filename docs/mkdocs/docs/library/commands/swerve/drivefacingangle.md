# DriveFacingAngle Command

## Overview
The DriveFacingAngle command implements sophisticated angle-controlled driving capabilities, allowing the robot to maintain a specific heading while moving. This command utilizes a ProfiledPIDController for precise angle management.

## Required Subsystems
- [SwerveDriveSubsystem](/5152_Template/library/subsystems/swerve/)

## Constructor Parameters
- `swerveDriveSubsystem`: The SwerveDrive subsystem instance this command will control
- `targetRotation`: A supplier that provides the target rotation angle

## Configuration Requirements
The following must be configured in TunerConstants:
- PID values for angle control
- Maximum angular velocity and acceleration constraints
- Profile constraints for smooth angle transitions

## API Reference
For detailed API documentation, see the [DriveFacingAngle Javadoc](/5152_Template/javadoc/frc/alotobots/library/commands/swervedrive/DriveFacingAngle.html)
