# 4. QuestNav + LocalizationFusion Quickstart Guide

## 4.1 Hardware Requirements

1. Meta Quest 3S headset (recommended)
2. USB-C to Ethernet adapter with power passthrough
3. 3D printed mount for the headset
4. Stable 5V power source (one of):
    - RoboRIO USB port (easiest)
    - 5V USB battery bank
    - 5V voltage regulator (e.g., Redux Robotics Zinc-V)

## 4.2 Software Setup

### 4.2.1 Install QuestNav App
1. Enable developer mode on your Quest headset:
    - Create/login to Meta developer account
    - Enable developer mode in Meta Quest app
    - Download Meta Quest Developer Hub (MQDH)

2. Configure Quest headset settings:
    - Enable travel mode
    - Set display timeout to 4 hours
    - Enable battery saver mode
    - Disable WiFi completely
    - Disable Bluetooth
    - Disable guardian system
        - Settings > Developer > Experimental Settings > Enable Custom Settings
        - Turn OFF: Physical Space Features, MTP Notification, Link Auto Connect

3. Install QuestNav:
    - Download the latest APK from [5152Alotobots/QuestNav](https://github.com/5152Alotobots/QuestNav)
    - Connect headset to computer via USB
    - Install using MQDH or ADB: `adb install QuestNav.apk`

### 4.2.2 Configure LocalizationFusion

1. Create necessary subsystem instances:
```java
public class RobotContainer {
    // IO interfaces
    private final OculusIO oculusIO = new OculusIO();
    private final AprilTagIO aprilTagIO = new AprilTagIO();

    // Vision subsystems
    private final OculusSubsystem oculusSubsystem = new OculusSubsystem(oculusIO);
    private final AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem(aprilTagIO);

    // Create pose sources
    private final OculusPoseSource oculusPoseSource = new OculusPoseSource(oculusSubsystem);
    private final AprilTagPoseSource aprilTagPoseSource = new AprilTagPoseSource(aprilTagSubsystem);

    // Auto chooser
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto");

    // Localization fusion
    private final LocalizationFusion localization = new LocalizationFusion(
        poseConsumer,    // Your pose consumer implementation
        oculusPoseSource,
        aprilTagPoseSource,
        autoChooser
    );
}
```

2. Configure physical setup in `OculusConstants.java`:
```java
public static final Transform2d ROBOT_TO_OCULUS = new Transform2d(
    0.075,              // X offset (meters, forward)
    0.0,                // Y offset (meters, left)
    new Rotation2d()    // Rotation (radians, CCW)
);
```

## 4.3 Operation

### 4.3.1 Pre-Match Setup
1. Power on Quest headset
2. Connect USB-Ethernet adapter
3. Launch QuestNav app
4. Verify in AdvantageScope:
    - Quest Ready indicator is green
    - Tags Ready indicator is green
    - System Ready indicator is green

### 4.3.2 During Match
- LocalizationFusion will automatically:
    - Use Quest as primary pose source
    - Fall back to AprilTags if Quest disconnects
    - Auto-realign using AprilTags when stationary
    - Handle pose resets and initialization

### 4.3.3 Troubleshooting
- If headset loses tracking:
    1. Ensure clear view of surroundings
    2. Check USB connection
    3. Verify power supply
    4. Use manual pose reset if needed:
```java
localization.requestResetOculusPoseViaAprilTags();
```

## 4.4 Best Practices
1. Mount headset securely with clear field view
2. Use stable 5V power source
3. Keep headset powered between matches
4. Monitor battery level
5. Monitor pose validation in AdvantageScope logging

## 4.5 Documentation & Configuration
For detailed documentation and configuration options, refer to:
- [LocalizationFusion Documentation](/5152_Reefscape/library/subsystems/vision/localizationfusion)
- [OculusSubsystem Documentation](/5152_Reefscape/library/subsystems/vision/oculus)

## 4.6 Additional Resources
- [QuestNav Repository](https://github.com/5152Alotobots/QuestNav)
- [Meta Quest Developer Hub](https://developer.oculus.com/downloads/package/oculus-developer-hub-win/)
- [Chief Delphi Support Thread](https://www.chiefdelphi.com/t/questnav-the-best-robot-pose-tracking-system-in-frc/476083)
