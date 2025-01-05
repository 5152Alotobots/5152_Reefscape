# DefaultDrive Command

## Overview
The DefaultDrive command implements the default driving behavior for the [SwerveDrive subsystem](../../subsystems/swerve.md). This command provides real-time velocity control based on driver inputs through the DriveCalculator utility.

## Required Subsystems
- [SwerveDriveSubsystem](/5152_Template/library/subsystems/swerve/)

## Constructor Parameters
- `swerveDriveSubsystem`: The SwerveDrive subsystem instance this command will control

## Configuration Requirements
- No additional configuration required beyond SwerveDrive subsystem configuration in TunerConstants
- Command uses drive calculations from SwerveDrive subsystem's configured settings

## API Reference
For detailed API documentation, see the [DefaultDrive Javadoc](/5152_Template/javadoc/frc/alotobots/library/commands/swervedrive/DefaultDrive.html)
