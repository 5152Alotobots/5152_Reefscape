# Robot Quickstart Guide

This guide provides step-by-step instructions for configuring and tuning your robot's swerve drive system.

## 1. Swerve Drive Configuration

### 1.1 CAN ID Management

> **Important**: Proper CAN ID management is critical for reliable robot operation
Before beginning any swerve configuration, verify all CAN IDs:

1. Check current CAN IDs in `Constants.java` under `CanId`:
   ```java
   public static class CanId {
       public static final int FRONT_LEFT_DRIVE_MTR_CAN_ID = 1;
       public static final int FRONT_LEFT_STEER_MTR_CAN_ID = 2;
       public static final int FRONT_LEFT_STEER_CAN_CODER_CAN_ID = 3;
       // ... etc
   }
   ```

2. If your physical devices have different IDs:
    - DO NOT change the IDs in code
    - Use Phoenix Tuner X to update the device IDs to match the code
    - This maintains backwards compatibility with previous configurations
    - Helps prevent confusion when sharing code or reverting changes

3. Best Practices:
    - Keep a CAN ID spreadsheet/document for your team
    - Label physical devices with their CAN IDs
    - Use Phoenix Tuner X's network view to verify all devices are visible
    - Test communication with each device before proceeding

4. Common Issues:
    - If devices aren't showing up, check CAN termination resistors
    - Verify firmware versions are up to date
    - Ensure power and CAN wire connections are secure

### 1.2 New Season Setup
When starting a new season:
1. Copy the previous year's tuner constants file (e.g., `TunerConstants2023.java`)
2. Rename it to match the new year (e.g., `TunerConstants2024.java`)
3. Update the class name in the file to match
4. Update any imports referencing the old class name
5. Update the robot's main Constants file:
   ```java
   // In Constants.java
   public static final TunerConstants tunerConstants = new TunerConstants2024();
   // or if using different configurations for simulation:
   public static final TunerConstants tunerConstants =
       Constants.currentMode == Mode.SIM ?
           new TunerConstantsSim() :
           new TunerConstants2024();
   ```
   This static field is used throughout the codebase to access swerve configurations.

6. Keep both files - old constants serve as a reference and backup

### 1.3 Module Configuration
The swerve drive configuration is generated using Phoenix Tuner X's Swerve Generator tool:

1. Launch Phoenix Tuner X and connect to the robot
2. Open the Swerve Generator tool
3. Configure module hardware in Phoenix Tuner X:
    - Select your motor type:
        - Kraken X60
        - Falcon 500
    - Set gear ratios for drive and steer (based on your swerve module type)
    - Input wheel radius
    - Select encoder type:
        - CANcoder
        - CANcoder 2.0

4. Set CAN IDs for each module:
    - Drive Motor
    - Turn Motor
    - CANcoder
    - Ensure IDs match physical hardware

5. Zero all CANcoders:
    - Point all wheels toward the center of the robot:
        - Bevel gears (black side) should face inward on all modules
        - If available, use the module's zeroing holes with a 3/16" bar
        - If no zeroing holes, align wheels visually toward robot center
    - Double check all wheels are properly aligned
    - Use Tuner X to record offsets
    - Verify offsets are correct by powering off/on and checking wheel alignment

6. Copying Generated Constants:
    - The generator creates a complete constants file with all swerve configurations
    - In your project, locate the `GeneratedConstants` inner class within your new `TunerConstants2024.java`
    - **DO NOT** replace the entire `TunerConstants2024.java` file
    - Only copy the constant values from the generator output into your existing `GeneratedConstants` class
    - This includes:
        - Module offsets
        - Gear ratios
        - PID gains
        - Basic configurations
        - Motor and encoder IDs
    - **IMPORTANT**: Delete or do not copy the `createSwerveDrive()` method
        - This method is not used in our implementation
        - We handle swerve drive creation differently
    - Optional: Replace hardcoded CAN IDs with Constants references
      ```java
      // Replace generated values like:
      public static final int kFrontLeftDriveMotorId = 1;

      // With references to Constants:
      public static final int kFrontLeftDriveMotorId = Constants.CanId.FRONT_LEFT_DRIVE_MTR_CAN_ID;
      ```
      This makes ID management more centralized and maintainable
    - Leave all custom configurations in `CustomConstants` unchanged
        - These values (like translation PID, rotation PID, constraints) will be tuned later
        - Do not overwrite them with generated values
    - The `TunerConstants2024.java` file implements `TunerConstants.java` and should maintain its structure

### 1.4 Drive System Testing
After configuring modules and before PID tuning, verify basic functionality:

1. Initial Movement Check:
    - Deploy code with default PID values:
      ```java
      // Default steer (turn) gains
      private static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.2)
          .withKV(1.59)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

      // Default drive gains
      private static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(0.1)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(0.124);
      ```
    - Lower robot onto ground
    - Very slowly test each movement:
        - Forward/Backward
        - Left/Right strafe
        - Rotation
        - **Note**: Movement may not be perfect yet as we use 254's Swerve Trajectory Setpoint Generator, which requires tuned PathPlanner constraints (we'll configure these in a later step)
    - Watch for:
        - All wheels spinning correct directions
        - No grinding or unusual noises
        - Basic responsiveness to commands

2. Module Behavior Verification:
    - Watch each module individually
    - Verify wheels point in expected directions
    - Make sure drive motors move in sync
    - Check for any modules "fighting" each other

### 1.5 Drive Characterization
Now we'll determine the feed-forward values:

1. Setup:
    - Place robot in an open space
    - Connect to robot via Driver Station
    - Open AdvantageScope

2. Run Characterization:
   Select the "Drive Simple FF Characterization" auto routine.
   Enable autonomous
   Disable after 10-15s
    - Command will gradually increase voltage
    - Watch modules for smooth acceleration
    - Let run until reaching full speed
    - Values will print to Driver Station console

3. Record Values:
    - Look for kS and kV output in console
    - Record both values for next step
    - Run 2-3 times to verify consistency

### 1.6 Drive Motor Tuning
Update PID and FF values based on characterization:

1. Initial Feed Forward:
   ```java
   // In GeneratedConstants:
   public static final Slot0Configs driveGains = new Slot0Configs()
       .withKS(/* value from characterization */)
       .withKV(/* value from characterization */)
       .withKA(0.01);  // Start small
   ```

2. Base PID Values:
   ```java
   public static final Slot0Configs driveGains = driveGains
       .withKP(0.05)  // Start conservative
       .withKI(0.0)   // Usually not needed
       .withKD(0.0);  // Add only if needed
   ```

3. Testing Process:
    - Deploy code
    - Start with very slow movements
    - Test forward/backward/strafe
    - Plot values for
      - /RealOutputs/SwerveStates/Measured
      - /RealOutputs/SwerveStates/SetpointsOptimized
    - Try to match the Measured value as close to Optimized as possible.
    - Transitions won't be instant, but ensure that we don't over/under shoot
    - Watch for:
        - Smooth acceleration
        - No oscillation
        - Wheels staying in sync

4. PID Tuning Steps:
   a. If response is sluggish:
    - Increase kP by 0.02
    - Test movement
    - Repeat until responsive

   b. If modules oscillate:
    - Reduce kP by 25%
    - Add small kD (start 0.001)
    - Test movement

   c. If wheels fight each other:
    - Verify module offsets
    - Check wheel direction conventions
    - Reduce kP slightly

5. Velocity Tuning:
    - Test at 25% speed:
        - Adjust kP until responsive
        - Add kD if oscillating
    - Test at 50% speed:
        - Verify stability
        - Adjust if needed
    - Test at full speed:
        - Final verification
        - Fine-tune as needed

6. Final Testing:
    - Run full-speed maneuvers
    - Check quick direction changes
    - Verify smooth acceleration
    - Test rotation stability
    - Look for any wheel slip

Your final drive gains might look like:
```java
public static final Slot0Configs driveGains = new Slot0Configs()
    .withKP(0.09)
    .withKI(0.0)
    .withKD(0.001)
    .withKS(0.19437)
    .withKV(0.75843)
    .withKA(0.01);
```

### 1.7 Measuring Maximum Speed
After tuning the drive motors, you'll need to measure the robot's actual maximum speed:

1. Safety First:
    - Place the robot securely on blocks
    - Ensure all wheels are free to spin
    - Clear the area around the robot
    - Have emergency stop ready

2. Using Phoenix Tuner X:
    - Connect to robot
    - Open "Plot" view
    - Select drive motors
    - Use "Control" tab
    - Set to "Voltage Control"
    - Command 12V to drive motors

3. Measuring Speed:
    - Let motors reach full speed
    - Record velocity from plot
    - Take measurements from all modules
    - Average the results
    - Convert to your preferred units (meters/second)

4. Updating Constants:
    - Open your TunerConstants file
    - Update the speed value:
    ```java
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.02);
    ```
    - This value is used for:
        - Path planning
        - Autonomous routines
        - Speed limiting
        - Controller scaling

5. Verification:
    - Deploy updated code
    - Test at various speed percentages
    - Verify autonomous paths work correctly
    - Check that speed limits are respected

### 1.8 Slip Current Measurement
After setting maximum speed, we need to determine the current limit that prevents wheel slip:

1. Setup:
    - Position robot against a solid wall
    - Open AdvantageScope
    - Create plots for:
        - Drive motor current (/Drive/Module.../DriveCurrentAmps)
        - Drive velocity (/Drive/Module.../DriveVelocityRadPerSec)

2. Measurement Process:
    - Gradually increase forward throttle
    - Watch velocity plot carefully
    - When velocity suddenly increases (wheel slip), note the current
    - This is your slip current threshold

3. Updating Constants:
    - Open your TunerConstants file
    - Set kSlipCurrent to the measured value:
    ```java
    public static final double kSlipCurrent = 40.0; // Amperes - adjust to your measured value
    ```
    - This prevents wheel slip during high-torque maneuvers

4. Verification:
    - Deploy updated code
    - Test aggressive movements
    - Verify wheels maintain traction
    - Check performance on different surfaces

### 1.9 Robot Mass Configuration
After setting up slip current limits, configure the robot's mass for PathPlanner:

1. Measure Robot Mass:
    - Weigh the complete robot
    - Include all components:
        - Battery
        - Bumpers
        - Game pieces (if carrying during auto)
    - Convert to kilograms

2. Update Configuration:
    - In your TunerConstants file, update the PathPlanner config:
    ```java
    public static final double ROBOT_MASS_KG = 34; // Update with your measured mass
    ```

### 1.10 Robot Moment of Inertia Measurement
After configuring robot mass, measure the robot's rotational inertia (MOI) for better path following:

1. Code Preparation:
    - Locate Module.java in your project
    - Find the runCharacterization method (around line 89)
    - Replace the existing method with:
    ```java
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d(constants.LocationX, constants.LocationY).plus(Rotation2d.kCCW_Pi_2));
    }
    ```
    - Deploy the updated code to the robot

2. Setup:
    - Clear a large, flat area
    - Place robot on smooth surface
    - Connect to robot via Driver Station
    - Open AdvantageScope for data logging

3. Data Collection:
    - Through dashboard autos, run each SysId routine:
        - Drive SysId (Quasistatic Forward)
        - Drive SysId (Quasistatic Reverse)
        - Drive SysId (Dynamic Forward)
        - Drive SysId (Dynamic Reverse)
    - Let each routine run for atleast 15s
    - Save logs after each run

4. Data Export:
    - Remove USB drive from robot
    - Connect to computer
    - Open AdvantageScope (v3.0.2 or later)
    - Load the log file
    - Go to "File" > "Export Data..."
    - Configure export settings:
        - Format: "WPILOG"
        - Timestamps: "AdvantageKit Cycles"
        - Select necessary fields for SysId
    - Save the converted log file

5. SysId Analysis:
    - Launch SysID
    - Load the exported log file
    - Record these values:
        - kA_angular (V/(rad/s²))
        - kA_linear (V/(m/s²))

6. Calculate MOI:
    Use this formula:
    ```
    MOI = mass * (trackwidth/2) * (kA_angular/kA_linear)
    ```
    Where:
    - mass = robot mass in kg
    - trackwidth = largest distance between wheel centers
    - kA_angular = angular acceleration feedforward
    - kA_linear = linear acceleration feedforward

7. Update Configuration:
    - In your TunerConstants file, update the MOI constant:
    ```java
    public static final double ROBOT_MOI = 4.24; // Update with calculated value
    ```
    This value represents the robot's resistance to rotational acceleration

### 1.11 Wheel Coefficient of Friction Measurement
After configuring mass and MOI, measure the wheel coefficient of friction for accurate path following:

1. Locate the wheel COF on the manufacturer website, some common examples have been provided here:
   - Most Colson wheels: 1.0
   - **BRAND NEW** Billet Wheel, 4"OD x 1.5"W (MK4/4i/4n): 1.1 **(THIS VALUE ONLY LASTS FOR 1-1.5 EVENTS WORTH OF USE)**
2. Update Configuration:
    - In your TunerConstants file, update the COF constant:
    ```java
    public static final double WHEEL_COF = 1.1;  // Update with measured value
    ```
    This value helps PathPlanner calculate maximum achievable accelerations

### 1.12 Turn Motor Tuning
The turn (steering) motors require different tuning approaches than drive motors:

1. Initial Setup:
    - Ensure wheels can rotate freely
    - Start with conservative values:
    ```java
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(40)
        .withKI(0)
        .withKD(0.1)
        .withKS(0.12)
        .withKV(0.102);
    ```

2. Position Control Testing:
    - Deploy code with initial values
    - Use driver station to command 90-degree turns
    - Watch for:
        - Quick response without overshooting
        - No oscillation at target position
        - Smooth acceleration and deceleration
        - Ability to hold position when pushed

3. PID Tuning Process:
   a. Start with Position Control:
    - Increase kP until wheels respond quickly
    - If oscillating, reduce kP by 25%
    - Add kD to dampen oscillations (start with kD = kP * 0.01)
    - Adjust until wheels snap to position without overshooting

   b. Add Feed Forward:
    - Start with small kS (0.1-0.2)
    - Increase if modules struggle to overcome friction
    - Add kV based on max velocity requirements
    - Keep kA at 0 unless needed for high acceleration

4. Common Issues:
    - Oscillation: Reduce kP or increase kD
    - Slow response: Increase kP
    - Position drift: Increase kS slightly
    - Overshooting: Increase kD or reduce kP
    - Grinding noise: Check mechanical alignment and reduce gains

5. Final Verification:
    - Test rapid direction changes
    - Verify holding position under load
    - Check for smooth motion at various speeds
    - Ensure all modules perform consistently

Your final turn gains might look like:
```java
public static final Slot0Configs steerGains = new Slot0Configs()
    .withKP(55)
    .withKI(0)
    .withKD(0.2)
    .withKS(0.12)
    .withKV(0.102)
    .withKA(0)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
```

### 1.13 PathPlanner PID Tuning
The final step in swerve drive configuration is tuning the PathPlanner translation and rotation PIDs:

1. Initial Values:
   These PIDs are defined in your TunerConstants file:
   ```java
   public static final PIDConstants translationPid = new PIDConstants(2.4, 0, 0.015);
   public static final PIDConstants rotationPid = new PIDConstants(7.8, 0, 0.015);
   ```

2. Tuning Process:
    - Open PathPlanner Desktop application
    - Select the Telemetry tab
    - Follow this sequence:
        1. Run "PP_StraightTest"
           - Adjust translationPid until actual path matches commanded path
           - Focus on matching the translation slopes as closely as possible
        2. Run "PP_RotationTest"
           - Tune rotationPid until rotation behavior is smooth and accurate
        3. Run "PP_HoloTest"
           - Final verification of both PIDs working together
           - Make minor adjustments if needed

3. Tips:
    - Start with P term only
    - Add D term to reduce oscillation
    - I term usually not needed
    - Test multiple times for consistency
    - Save values after successful tuning

### 1.14 Final Verification Testing
Before considering the swerve drive fully configured, perform these comprehensive tests:

1. Teleoperated Testing:
    - Drive the robot in all directions:
        - Forward/backward at varying speeds
        - Left/right strafing
        - Diagonal movements
        - Rotation while moving (both directions)
    - Check for:
        - Smooth acceleration/deceleration
        - No unexpected jerking or hesitation
        - Accurate response to joystick inputs
        - Consistent behavior at all speeds
        - No wheel scrubbing during rotation
        - Proper tracking during diagonal movement

2. Autonomous Path Testing:
    - Run several test paths:
        - Straight lines
        - S-curves
        - Figure-8 patterns
        - Complex multi-part paths
    - Verify:
        - Robot follows commanded path closely
        - No significant corner cutting
        - Smooth transitions between path segments
        - Consistent speed throughout paths
        - Accurate final positioning
        - Repeatable results across multiple runs

3. Edge Case Testing:
    - Quick direction changes
    - Full-speed emergency stops
    - Operation near physical obstacles
    - Movement on different surface materials
    - Performance with varying battery voltage
    - Behavior under maximum load conditions

Note: While the robot should now follow paths accurately, some remaining inaccuracies are normal and will be corrected later using vision-based localization systems.

## 2. Vision Coprocessor Setup

### 2.1 Initial Hardware Setup

1. Coprocessor Preparation:
    - Download latest Orange PI 5 PhotonVision image
    - Flash image to microSD card
    - Insert microSD into Orange PI 5

2. Camera Naming:
    - Download [ArducamUVCSerialNumber_Official.zip](https://www.arducam.com/wp-content/uploads/2023/10/ArducamUVCSerialNumber_Official.zip)
    - Extract the program
    - For each Arducam OV9281 camera:
        1. Connect camera to laptop via USB
        2. Open ArducamUVCSerialNumber program
        3. In "Device name" field, enter camera position:
            - Format: `POSITION_AprilTag`
            - Examples: `FL_AprilTag`, `FM_AprilTag`, `FR_AprilTag`
        4. Click "Write"
        5. Open Device Manager
        6. Uninstall the camera
        7. Reconnect the camera
        8. Verify new name appears
        9. Disconnect from laptop
        10. Connect to Orange PI 5
    - Repeat for all cameras

### 2.2 PhotonVision Configuration

1. Pipeline Setup:
    - Open PhotonVision web interface
    - For each camera:
        1. Select camera in UI
        2. Create new pipeline named "AprilTag"
        3. Set pipeline type to "AprilTag"
        4. Configure processing:
            - Decimate: 2
            - Blur: 1
            - Auto White Balance: ON
        5. Set camera settings:
            - Resolution: 1280x720
            - FPS: 100 (or similar)
            - Stream Resolution: Lowest available
        6. Navigate to Cameras tab
        7. Select camera
        8. Set Model as "OV9281"

2. Camera Calibration:
    - For each camera:
        1. Select calibration settings:
            - Tag Family: 5x5
            - Resolution: 720p
            - Pattern Spacing: 3.15 inches
            - Marker Size: 2.36 inches
            - Board: 12x8
        2. Take multiple calibration snapshots:
            - Vary angles and distances
            - Include corner views
            - Mix close and far positions
        3. Run calibration
        4. Verify mean error < 1.0 pixels
        5. If error too high:
            - Delete poor quality snapshots
            - Add more varied angles
            - Recalibrate

3. Final Configuration:
    - For each pipeline:
        1. Enable 3D mode
        2. Enable multi-tag detection
        3. Save settings

4. Field Tuning:
    - At competition field:
        - Adjust exposure
        - Tune brightness
        - Set appropriate gain
        - Test detection reliability
        - Save field-specific settings

## 3. AprilTag Vision Configuration

### 2.1 Camera Offset Measurement

Accurate camera position measurements are critical for AprilTag vision:

1. Physical Measurements:
    - Use calipers or precise measuring tools
    - Measure from robot center (origin) to camera lens center
    - Record three distances for each camera:
        - Forward distance (X): positive towards robot front
        - Left distance (Y): positive towards robot left
        - Up distance (Z): positive towards robot top
    - Measure camera angles:
        - Pitch: downward tilt (usually negative)
        - Yaw: left/right rotation

2. Update Constants:
    ```java
    private static final Transform3d[] CAMERA_OFFSETS = new Transform3d[] {
        // Front Left Camera
        new Transform3d(
            new Translation3d(0.245, 0.21, 0.17),  // X, Y, Z in meters
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(45))),  // Roll, Pitch, Yaw

        // Front Middle Camera
        new Transform3d(
            new Translation3d(0.275, 0.0, 0.189),
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(0)))
    };
    ```

3. Camera Configuration:
    ```java
    // Add configuration for each physical camera
    private static final CameraConfig[] CAMERA_CONFIGS = new CameraConfig[] {
        new CameraConfig("FL_AprilTag", CAMERA_OFFSETS[0], new SimCameraProperties()),
        new CameraConfig("FM_AprilTag", CAMERA_OFFSETS[1], new SimCameraProperties())
    };
    ```

4. IO Configuration:
    ```java
    // In RobotContainer.java constructor:
    aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[0]),
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[1]));
    ```

5. Replay Support:
    ```java
    // For replay mode, match array size to physical cameras
    aprilTagSubsystem =
            new AprilTagSubsystem(
                swerveDriveSubsystem::addVisionMeasurement,
                new AprilTagIO() {},
                new AprilTagIO() {});
    ```

### 2.2 Camera Verification

After configuring camera offsets:

1. Physical Checks:
    - Verify camera mounts are secure
    - Check USB connections
    - Confirm cameras are powered
    - LED indicators should be on

2. Network Verification:
    - Open PhotonVision dashboard
    - Confirm all cameras are connected
    - Check video feeds are active
    - Verify camera names match configs

3. Basic Testing:
    - Hold AprilTag in camera view
    - Confirm detection in dashboard
    - Check pose estimation quality
    - Verify reasonable distance estimates

4. Optional Camera Tuning:
    - Camera Trust Factors:
        ```java
        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        // SHOULD NEVER BE LESS THAN 1.0, NUMBERS GREATER THAN 1 = TRUST LESS
        public static double[] CAMERA_STD_DEV_FACTORS = new double[] {1.0, 1.0};
        ```
        - Higher values = trust that camera less
        - Example: `{1.0, 1.5}` trusts first camera more than second
        - Never use values less than 1.0
        - Useful when some cameras are more reliable than others

    - Ambiguity Filtering:
        ```java
        public static double MAX_AMBIGUITY = 0.3;
        ```
        - Filters out less confident tag detections
        - Lower values = stricter filtering
        - Range 0.0 to 1.0
        - Start with 0.3 and adjust based on false positive rate

5. Common Issues:
    - Camera disconnections: Check USB connections
    - Poor detection: Adjust exposure/brightness
    - Incorrect poses: Double-check offset measurements
    - Network lag: Monitor bandwidth usage

## 3. Object Detection Configuration

### 3.1 Initial Hardware Setup

1. Camera Naming:
    - Download [ArducamUVCSerialNumber_Official.zip](https://www.arducam.com/wp-content/uploads/2023/10/ArducamUVCSerialNumber_Official.zip)
    - Extract the program
    - For each Arducam OV9782 camera:
        1. Connect camera to laptop via USB
        2. Open ArducamUVCSerialNumber program
        3. In "Device name" field, enter camera position:
            - Format: `POSITION_Object`
            - Examples: `FL_Object`, `FM_Object`, `FR_Object`
        4. Click "Write"
        5. Open Device Manager
        6. Uninstall the camera
        7. Reconnect the camera
        8. Verify new name appears
        9. Disconnect from laptop
        10. Connect to Orange PI 5
    - Repeat for all cameras

### 3.2 PhotonVision Configuration

1. Pipeline Setup:
    - Open PhotonVision web interface
    - For each camera:
        1. Select camera in UI
        2. Create new pipeline named "ObjectDetection"
        3. Set pipeline type to "ObjectDetection"
        4. Configure processing:
            - Auto White Balance: ON
        5. Set camera settings:
            - Resolution: 1280x720
            - FPS: 30 (or similar)
            - Stream Resolution: Lowest available
        6. Navigate to Cameras tab
        7. Select camera
        8. Set Model as "OV9782"

2. Camera Calibration:
    - For each camera:
        1. Select calibration settings:
            - Tag Family: 5x5
            - Resolution: 720p
            - Pattern Spacing: 3.15 inches
            - Marker Size: 2.36 inches
            - Board: 12x8
        2. Take multiple calibration snapshots:
            - Vary angles and distances
            - Include corner views
            - Mix close and far positions
        3. Run calibration
        4. Verify mean error < 1.0 pixels
        5. If error too high:
            - Delete poor quality snapshots
            - Add more varied angles
            - Recalibrate

3. Field Tuning:
    - At competition field:
        - Adjust exposure
        - Tune brightness
        - Set appropriate gain
        - Test detection reliability
        - Save field-specific settings

### 3.3 Camera Offset Measurement

Accurate camera position measurements are critical for object detection:

1. Physical Measurements:
    - Use calipers or precise measuring tools
    - Measure from robot center (origin) to camera lens center
    - Record three distances for each camera:
        - Forward distance (X): positive towards robot front
        - Left distance (Y): positive towards robot left
        - Up distance (Z): positive towards robot top
    - Measure camera angles:
        - Pitch: downward tilt (usually negative)
        - Yaw: left/right rotation

2. Update Constants:
    ```java
    private static final Transform3d[] CAMERA_OFFSETS =
      new Transform3d[] {
        // Front Middle
        new Transform3d(
            new Translation3d(0.275, 0.0, 0.23),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))
      };
    ```

3. Camera Configuration:
    ```java
    // Add configuration for each physical camera
    public static final CameraConfig[] CAMERA_CONFIGS = {
    new CameraConfig(
        "FM_ObjectDetection",
        CAMERA_OFFSETS[0],
        new SimCameraProperties())
   };
    ```

4. IO Configuration:
    ```java
    // In RobotContainer.java constructor:
    objectDetectionSubsystem =
            new ObjectDetectionSubsystem(
                swerveDriveSubsystem::getPose,
                new ObjectDetectionIOPhotonVision(ObjectDetectionConstants.CAMERA_CONFIGS[0]));
    ```

5. Replay Support:
    ```java
    // For replay mode, match array size to physical cameras
    objectDetectionSubsystem =
            new ObjectDetectionSubsystem(swerveDriveSubsystem::getPose, new ObjectDetectionIO() {});
    ```

### 3.4 Game Element Configuration

Before testing object detection, configure the game elements that need to be detected:

1. Define Game Elements:
    ```java
    // In GameElementConstants.java
    // All measurements in meters
    public static final GameElement NOTE = new GameElement("Note", 0.36, 0.36, 0.05);
    ```

2. Create Class ID Array:
    ```java
    // Game elements array indexed by class ID
    // IMPORTANT: Order must match neural network model's class IDs
    public static final GameElement[] GAME_ELEMENTS = new GameElement[] {
        NOTE      // Class ID 0
    };
    ```

3. Important Considerations:
    - Array indices must match model's class IDs exactly
    - Measurements must be in meters
    - Dimensions are width, length, height
    - Names should match what's shown in PhotonVision

### 3.5 Camera Verification

After configuring cameras and game elements:

1. Physical Checks:
    - Verify camera mounts are secure
    - Check USB connections
    - Confirm cameras are powered
    - LED indicators should be on

2. Network Verification:
    - Open PhotonVision dashboard
    - Confirm all cameras are connected
    - Check video feeds are active
    - Verify camera names match configs

3. Basic Testing:
    - Place game piece in camera view
    - Confirm detection in dashboard
    - Check pose estimation quality
    - Verify reasonable distance estimates

4. Filtering Configuration:
    - Position Match Tolerance:
        ```java
        // Tolerance in meters for matching object positions
        // Default is usually fine, but can be adjusted if needed
        public static final double POSITION_MATCH_TOLERANCE = 0.5;
        ```
        - Larger values: More stable tracking during rotation
        - Smaller values: More accurate position tracking
        - Trade-off between stability and accuracy
        - Start with default and adjust if objects appear unstable

5. Common Issues:
    - Camera disconnections: Check USB connections
    - Poor detection: Adjust exposure/brightness
    - Incorrect poses: Double-check offset measurements
    - Network lag: Monitor bandwidth usage
    - Unstable tracking: Try increasing POSITION_MATCH_TOLERANCE
    - Position jumps: Try decreasing POSITION_MATCH_TOLERANCE

Congratulations! Your project should now be fully configured and tuned for optimal performance.
