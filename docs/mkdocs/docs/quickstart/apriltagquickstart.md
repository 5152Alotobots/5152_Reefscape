## 2. Vision Coprocessor Setup

### 2.1 Initial Hardware Setup

1. Coprocessor Preparation:
   - Make sure your coprocessor is already set up from a previous quickstart, OR
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
        -
### 2.3 Camera Offset Measurement

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

### 2.4 Camera Verification

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