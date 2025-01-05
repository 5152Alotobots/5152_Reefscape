# Oculus Navigation Subsystem

The Oculus Navigation subsystem provides robot positioning and orientation tracking using an Oculus Quest VR headset. This system enables high-precision robot navigation and positioning by leveraging the Oculus Quest's advanced tracking capabilities.

## Constructor and Configuration

The subsystem is constructed with an OculusIO interface implementation:

```java
public OculusSubsystem(OculusIO io)
```

The io parameter can be either:
- `OculusIOReal` for real hardware operation
- `OculusIOSim` for simulation testing

## Commands

The following commands interact with the Oculus subsystem:

- [Ping Command](/5152_Template/library/commands/questnav/pingcommand) - Tests communication with the headset
- [Reset Pose Command](/5152_Template/library/commands/questnav/resetposecommand) - Resets the headset's position tracking
- [Zero Heading Command](/5152_Template/library/commands/questnav/zeroheadingcommand) - Zeros the headset's rotation tracking

## Required Configuration

1. Software Setup:
    - [QuestNav](https://github.com/juchong/QuestNav/tree/main) must be deployed and running on the Oculus Quest headset

2. Physical Setup:
    - Oculus Quest headset must be mounted securely to the robot
    - Headset position relative to robot center must be configured in OculusConstants.OCULUS_TO_ROBOT

## Reference Documentation

Full JavaDoc reference: [Package Documentation](/5152_Template/javadoc/frc/alotobots/library/subsystems/vision/questnav/package-summary.html)
