## 3. Object Detection Configuration

### 3.1 Initial Hardware Setup

1. Coprocessor Preparation:
    - Make sure your coprocessor is already set up from a previous quickstart, OR
    - Download latest Orange PI 5 PhotonVision image
    - Flash image to microSD card
    - Insert microSD into Orange PI 5

2. Camera Naming:
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
