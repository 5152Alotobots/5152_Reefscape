# Swerve Drive Characterization Commands

## Overview
The characterization commands provide system identification capabilities for the swerve drive subsystem. These commands enable precise measurement and calculation of critical system parameters.

## Required Subsystems
- [SwerveDriveSubsystem](/5152_Template/library/subsystems/swerve/)

## Commands

### WheelRadiusCharacterization
Measures and calculates the effective wheel radius of the swerve modules.

#### Parameters
- `swerveDriveSubsystem`: The SwerveDrive subsystem instance to characterize

#### Output
Provides:
- Wheel radius measurement in meters and inches
- Wheel and gyro delta measurements
- Calculation confidence metrics

### FeedforwardCharacterization
Determines the feedforward characterization constants for the drive motors.

#### Parameters
- `swerveDriveSubsystem`: The SwerveDrive subsystem instance to characterize

#### Output
Provides:
- kS (static friction coefficient)
- kV (velocity coefficient)
- Data points for validation

## Configuration Requirements
- Clear operating space required
- Robot must be on a level surface
- Battery should be fully charged
- Robot must start in a known orientation

## API Reference
For detailed API documentation, see the [Characterization Commands Javadoc](/5152_Template/javadoc/frc/alotobots/library/commands/swervedrive/characterization/)
