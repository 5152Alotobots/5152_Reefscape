# Wrist Subsystem

A subsystem that controls a single-jointed wrist mechanism capable of rotating to specific angles for game piece manipulation.

## Constructor Details
```java
public WristSubsystem(WristIO io)
```
- `io`: Hardware abstraction interface for the wrist. Can be either:
    - `WristIOTalonFXReal`: For real robot hardware using TalonFX motor controller
    - `WristIOTalonFXSim`: For simulation using WPILib's physics simulation

## Commands
The wrist subsystem is used by the following commands:
- [DefaultWristOpenLoop](/5152_Reefscape/library/commands/wrist/defaultwristopenloop)
- [WristRunToAngle](/5152_Reefscape/library/commands/wrist/wristruntoangle)

## Configuration Requirements
1. Hardware Configuration:
    - TalonFX motor controller with proper CAN ID
    - CANCoder absolute encoder with proper CAN ID
    - Proper gear ratio configuration (ROTOR_TO_SENSOR_RATIO)
    - Correct motor and encoder direction settings

2. Control Parameters:
    - PID gains for both velocity and position control modes
    - Current limits and safety thresholds
    - Motion constraints (min/max angles, speed limits)
    - Gravity compensation parameters

3. Setpoint Configurations:
    - Various angle presets for different positions (stowed, ground intake, scoring positions)
    - Proper absolute encoder offset calibration

## Reference Documentation
- [Wrist Subsystem Javadoc](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/wrist/package-summary.html)
