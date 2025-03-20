/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.photonvision.apriltag;

import static frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIO;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIOInputsAutoLogged;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * AprilTagSubsystem handles detection of AprilTag fiducial markers for robot localization.
 *
 * <p>This subsystem processes data from one or more cameras to detect AprilTags in the environment
 * and estimate the robot's position on the field. It supports both multi-tag and single-tag
 * detection with appropriate uncertainty estimation.
 *
 * <p>Multi-tag detection (seeing multiple tags simultaneously) is generally more accurate and is
 * preferred when available, while single-tag detection provides fallback position estimation with
 * higher uncertainty.
 */
public class AprilTagSubsystem extends SubsystemBase {

  // IO and inputs for each camera
  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;

  // Tracking for multi-tag pose validity and timing
  private final boolean[] hasValidMultiTagPose;
  private final double[] lastMultiTagPoseTimestamp;

  // Single tag poses are tracked separately but not returned by getCurrentPose
  private final boolean[] hasValidSingleTagPose;
  private final double[] lastSingleTagPoseTimestamp;

  // Consumer for applying vision measurements to odometry/pose estimation
  private final AprilTagConsumer aprilTagConsumer;

  /**
   * Creates a new AprilTagSubsystem with the given consumer and IO interfaces.
   *
   * @param aprilTagConsumer Consumer that will receive vision measurement updates
   * @param io One or more AprilTagIO objects representing camera inputs
   */
  public AprilTagSubsystem(AprilTagConsumer aprilTagConsumer, AprilTagIO... io) {
    this.io = io;
    this.aprilTagConsumer = aprilTagConsumer;
    this.inputs = new AprilTagIOInputsAutoLogged[io.length];

    // Initialize inputs array for each camera
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagIOInputsAutoLogged();
    }

    // Verify configuration values are valid
    validateConfiguration();

    // Initialize tracking arrays for pose validity and timestamps
    hasValidMultiTagPose = new boolean[inputs.length];
    hasValidSingleTagPose = new boolean[inputs.length];
    lastMultiTagPoseTimestamp = new double[inputs.length];
    lastSingleTagPoseTimestamp = new double[inputs.length];

    // Set initial values
    for (int i = 0; i < inputs.length; i++) {
      hasValidMultiTagPose[i] = false;
      hasValidSingleTagPose[i] = false;
      lastMultiTagPoseTimestamp[i] = -1;
      lastSingleTagPoseTimestamp[i] = -1;
    }
  }

  /**
   * Periodic update method - called repeatedly by the command scheduler.
   *
   * <p>This method handles:
   *
   * <ul>
   *   <li>Reading camera data from all connected cameras
   *   <li>Processing tag detections to estimate robot poses
   *   <li>Filtering out invalid or unreliable detections
   *   <li>Providing position updates to the AprilTagConsumer
   *   <li>Logging data for debugging and analysis
   * </ul>
   */
  @Override
  public void periodic() {
    // Update inputs and check camera connections
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/AprilTag/" + CAMERA_CONFIGS[i].name(), inputs[i]);
    }

    // Initialize logging value lists
    List<Pose3d> allMultiTagPoses = new LinkedList<>();
    List<Pose3d> allMultiTagRobotPoses = new LinkedList<>();
    List<Pose3d> allMultiTagRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allMultiTagRobotPosesRejected = new LinkedList<>();

    List<Pose3d> allSingleTagPoses = new LinkedList<>();
    List<Pose3d> allSingleTagRobotPoses = new LinkedList<>();
    List<Pose3d> allSingleTagRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allSingleTagRobotPosesRejected = new LinkedList<>();

    // Process camera data and populate logging lists
    processCameraData(
        allMultiTagPoses,
        allMultiTagRobotPoses,
        allMultiTagRobotPosesAccepted,
        allMultiTagRobotPosesRejected,
        allSingleTagPoses,
        allSingleTagRobotPoses,
        allSingleTagRobotPosesAccepted,
        allSingleTagRobotPosesRejected);

    // Log summary data across all cameras
    logSummaryData(
        allMultiTagPoses,
        allMultiTagRobotPoses,
        allMultiTagRobotPosesAccepted,
        allMultiTagRobotPosesRejected,
        allSingleTagPoses,
        allSingleTagRobotPoses,
        allSingleTagRobotPosesAccepted,
        allSingleTagRobotPosesRejected);

    // Log time since last pose for each camera
    for (int i = 0; i < inputs.length; i++) {
      Logger.recordOutput(
          "Vision/AprilTag/" + CAMERA_CONFIGS[i].name() + "/MultiTag/TimeSinceLastPose",
          Timer.getTimestamp() - lastMultiTagPoseTimestamp[i]);

      Logger.recordOutput(
          "Vision/AprilTag/" + CAMERA_CONFIGS[i].name() + "/SingleTag/TimeSinceLastPose",
          Timer.getTimestamp() - lastSingleTagPoseTimestamp[i]);
    }
  }

  /**
   * Validates that the camera standard deviation factors are within acceptable ranges.
   *
   * @throws IllegalArgumentException if any factor is less than 1.0
   */
  private void validateConfiguration() {
    for (double factor : CAMERA_STD_DEV_FACTORS) {
      if (factor < 1.0) {
        throw new IllegalArgumentException(
            "[AprilTagSubsystem] STD factor must be >= 1.0, but was: " + factor);
      }
    }
  }

  /**
   * Processes data from all cameras, identifies valid measurements, and updates pose estimates.
   *
   * @param allMultiTagPoses List to populate with detected multi-tag poses
   * @param allMultiTagRobotPoses List to populate with robot poses from multi-tag detection
   * @param allMultiTagRobotPosesAccepted List to populate with accepted multi-tag robot poses
   * @param allMultiTagRobotPosesRejected List to populate with rejected multi-tag robot poses
   * @param allSingleTagPoses List to populate with detected single-tag poses
   * @param allSingleTagRobotPoses List to populate with robot poses from single-tag detection
   * @param allSingleTagRobotPosesAccepted List to populate with accepted single-tag robot poses
   * @param allSingleTagRobotPosesRejected List to populate with rejected single-tag robot poses
   */
  private void processCameraData(
      List<Pose3d> allMultiTagPoses,
      List<Pose3d> allMultiTagRobotPoses,
      List<Pose3d> allMultiTagRobotPosesAccepted,
      List<Pose3d> allMultiTagRobotPosesRejected,
      List<Pose3d> allSingleTagPoses,
      List<Pose3d> allSingleTagRobotPoses,
      List<Pose3d> allSingleTagRobotPosesAccepted,
      List<Pose3d> allSingleTagRobotPosesRejected) {

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Split tag poses by type
      List<Pose3d> multiTagPoses = new LinkedList<>();
      List<Pose3d> singleTagPoses = new LinkedList<>();

      // Split robot poses by type
      List<Pose3d> multiTagRobotPoses = new LinkedList<>();
      List<Pose3d> multiTagRobotPosesAccepted = new LinkedList<>();
      List<Pose3d> multiTagRobotPosesRejected = new LinkedList<>();

      List<Pose3d> singleTagRobotPoses = new LinkedList<>();
      List<Pose3d> singleTagRobotPosesAccepted = new LinkedList<>();
      List<Pose3d> singleTagRobotPosesRejected = new LinkedList<>();

      // Process tag poses with separated lists
      processTagPoses(cameraIndex, multiTagPoses, singleTagPoses);

      // Process multi-tag observations
      processMultiTagPoseObservations(
          cameraIndex, multiTagRobotPoses, multiTagRobotPosesAccepted, multiTagRobotPosesRejected);

      // Process single-tag observations
      processSingleTagPoseObservations(
          cameraIndex,
          singleTagRobotPoses,
          singleTagRobotPosesAccepted,
          singleTagRobotPosesRejected);

      // Log camera data with separated lists
      logCameraData(
          cameraIndex,
          multiTagPoses,
          singleTagPoses,
          multiTagRobotPoses,
          multiTagRobotPosesAccepted,
          multiTagRobotPosesRejected,
          singleTagRobotPoses,
          singleTagRobotPosesAccepted,
          singleTagRobotPosesRejected);

      // Accumulate all results
      allMultiTagPoses.addAll(multiTagPoses);
      allMultiTagRobotPoses.addAll(multiTagRobotPoses);
      allMultiTagRobotPosesAccepted.addAll(multiTagRobotPosesAccepted);
      allMultiTagRobotPosesRejected.addAll(multiTagRobotPosesRejected);

      allSingleTagPoses.addAll(singleTagPoses);
      allSingleTagRobotPoses.addAll(singleTagRobotPoses);
      allSingleTagRobotPosesAccepted.addAll(singleTagRobotPosesAccepted);
      allSingleTagRobotPosesRejected.addAll(singleTagRobotPosesRejected);
    }
  }

  /**
   * Processes tag IDs from a specific camera, retrieving their field poses from the AprilTag
   * layout.
   *
   * @param cameraIndex Index of the camera to process
   * @param multiTagPoses List to populate with multi-tag poses
   * @param singleTagPoses List to populate with single-tag poses
   */
  private void processTagPoses(
      int cameraIndex, List<Pose3d> multiTagPoses, List<Pose3d> singleTagPoses) {

    // Process multi-tag IDs
    for (int tagId : inputs[cameraIndex].multiTagIds) {
      var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
      tagPose.ifPresent(multiTagPoses::add);
    }

    // Process single tag IDs
    for (int tagId : inputs[cameraIndex].singleTagIds) {
      var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
      tagPose.ifPresent(singleTagPoses::add);
    }
  }

  /**
   * Processes multi-tag observations from a camera, filtering valid poses and updating position.
   *
   * @param cameraIndex Index of the camera to process
   * @param robotPoses List to populate with all multi-tag robot poses
   * @param robotPosesAccepted List to populate with accepted multi-tag robot poses
   * @param robotPosesRejected List to populate with rejected multi-tag robot poses
   */
  private void processMultiTagPoseObservations(
      int cameraIndex,
      List<Pose3d> robotPoses,
      List<Pose3d> robotPosesAccepted,
      List<Pose3d> robotPosesRejected) {

    for (var observation : inputs[cameraIndex].multiTagObservations) {
      // Check if pose should be rejected based on criteria
      boolean rejectPose = shouldRejectMultiTagPose(observation);

      // Add to all poses list
      robotPoses.add(observation.pose());

      // If rejected, add to rejected list and skip processing
      if (rejectPose) {
        robotPosesRejected.add(observation.pose());
        continue;
      }

      // Add to accepted list
      robotPosesAccepted.add(observation.pose());

      // Update pose estimation with this observation
      updatePoseFromMultiTag(observation, cameraIndex);

      // Mark this camera as having a valid multi-tag pose
      hasValidMultiTagPose[cameraIndex] = true;
      lastMultiTagPoseTimestamp[cameraIndex] = observation.timestamp();
    }
  }

  /**
   * Processes single-tag observations from a camera, filtering valid poses and updating position.
   *
   * @param cameraIndex Index of the camera to process
   * @param robotPoses List to populate with all single-tag robot poses
   * @param robotPosesAccepted List to populate with accepted single-tag robot poses
   * @param robotPosesRejected List to populate with rejected single-tag robot poses
   */
  private void processSingleTagPoseObservations(
      int cameraIndex,
      List<Pose3d> robotPoses,
      List<Pose3d> robotPosesAccepted,
      List<Pose3d> robotPosesRejected) {

    for (var observation : inputs[cameraIndex].singleTagObservations) {
      // Check if pose should be rejected based on criteria
      boolean rejectPose = shouldRejectSingleTagPose(observation);

      // Create pose with Z=0 (robot on the ground)
      Pose3d singleTagPose =
          new Pose3d(
              observation.pose().getX(),
              observation.pose().getY(),
              0.0,
              new Rotation3d(observation.pose().getRotation()));

      // Add to all poses list
      robotPoses.add(singleTagPose);

      // If rejected, add to rejected list and skip processing
      if (rejectPose) {
        robotPosesRejected.add(singleTagPose);
        continue;
      }

      // Add to accepted list
      robotPosesAccepted.add(singleTagPose);

      // Update single tag pose data, but don't use it for getCurrentPose()
      updatePoseFromSingleTag(observation, cameraIndex);
      hasValidSingleTagPose[cameraIndex] = true;
      lastSingleTagPoseTimestamp[cameraIndex] = observation.timestamp();
    }
  }

  /**
   * Determines if a multi-tag pose observation should be rejected.
   *
   * @param observation The multi-tag observation to evaluate
   * @return True if the pose should be rejected, false otherwise
   */
  private boolean shouldRejectMultiTagPose(AprilTagIO.MultiTagObservation observation) {
    return observation.tagCount() == 0
        || (observation.tagCount() == 1 && observation.ambiguity() > MULTITAG_MAX_AMBIGUITY)
        || Math.abs(observation.pose().getZ()) > MAX_Z_ERROR
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();
  }

  /**
   * Determines if a single-tag pose observation should be rejected.
   *
   * @param observation The single-tag observation to evaluate
   * @return True if the pose should be rejected, false otherwise
   */
  private boolean shouldRejectSingleTagPose(AprilTagIO.SingleTagObservation observation) {
    return observation.ambiguity() > SINGLE_TAG_MAX_AMBIGUITY
        || observation.tagDistance() > SINGLE_TAG_MAX_DISTANCE
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();
  }

  /**
   * Updates the robot's pose estimation based on a multi-tag observation.
   *
   * @param observation The multi-tag observation to use for the update
   * @param cameraIndex Index of the camera that made the observation
   */
  private void updatePoseFromMultiTag(AprilTagIO.MultiTagObservation observation, int cameraIndex) {
    // Calculate standard deviation based on tag distance and count
    // Uncertainty increases with distance squared and decreases with more tags
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = MULTI_TAG_LINEAR_STD_DEV_BASE * stdDevFactor;
    double angularStdDev = MULTI_TAG_ANGULAR_STD_DEV_BASE * stdDevFactor;

    // Apply camera-specific adjustment factor if available
    if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
      linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
      angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
    }

    // Create standard deviation matrix
    Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);

    // Send pose to consumer
    aprilTagConsumer.accept(
        observation.pose().toPose2d(), observation.timestamp(), multiTagStdDevs);
  }

  /**
   * Updates the robot's pose estimation based on a single-tag observation.
   *
   * @param observation The single-tag observation to use for the update
   * @param cameraIndex Index of the camera that made the observation
   */
  private void updatePoseFromSingleTag(
      AprilTagIO.SingleTagObservation observation, int cameraIndex) {
    // Higher uncertainty for single tag observations - increases with distance squared
    double stdDevFactor = Math.pow(observation.tagDistance(), 2.0);
    double linearStdDev = SINGLE_TAG_LINEAR_STD_DEV_BASE * stdDevFactor;
    double angularStdDev = SINGLE_TAG_ANGULAR_STD_DEV_BASE * stdDevFactor;

    // Apply camera-specific adjustment factor if available
    if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
      linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
      angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
    }

    // Create standard deviation matrix
    Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);

    // Accept poses
    aprilTagConsumer.accept(observation.pose(), observation.timestamp(), singleTagStdDevs);
  }

  /**
   * Logs pose data from a specific camera for debugging and analysis.
   *
   * @param cameraIndex Index of the camera being logged
   * @param multiTagPoses List of multi-tag poses detected
   * @param singleTagPoses List of single-tag poses detected
   * @param multiTagRobotPoses List of robot poses from multi-tag detection
   * @param multiTagRobotPosesAccepted List of accepted multi-tag robot poses
   * @param multiTagRobotPosesRejected List of rejected multi-tag robot poses
   * @param singleTagRobotPoses List of robot poses from single-tag detection
   * @param singleTagRobotPosesAccepted List of accepted single-tag robot poses
   * @param singleTagRobotPosesRejected List of rejected single-tag robot poses
   */
  private void logCameraData(
      int cameraIndex,
      List<Pose3d> multiTagPoses,
      List<Pose3d> singleTagPoses,
      List<Pose3d> multiTagRobotPoses,
      List<Pose3d> multiTagRobotPosesAccepted,
      List<Pose3d> multiTagRobotPosesRejected,
      List<Pose3d> singleTagRobotPoses,
      List<Pose3d> singleTagRobotPosesAccepted,
      List<Pose3d> singleTagRobotPosesRejected) {

    String cameraPrefix = "Vision/AprilTag/" + CAMERA_CONFIGS[cameraIndex].name();

    // Log multi-tag data
    Logger.recordOutput(cameraPrefix + "/MultiTag/Poses", multiTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/MultiTag/RobotPoses", multiTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/MultiTag/RobotPosesAccepted",
        multiTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/MultiTag/RobotPosesRejected",
        multiTagRobotPosesRejected.toArray(new Pose3d[0]));

    // Log single-tag data
    Logger.recordOutput(cameraPrefix + "/SingleTag/Poses", singleTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/SingleTag/RobotPoses", singleTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/SingleTag/RobotPosesAccepted",
        singleTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/SingleTag/RobotPosesRejected",
        singleTagRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /**
   * Logs summary data across all cameras for debugging and analysis.
   *
   * @param allMultiTagPoses Combined list of multi-tag poses from all cameras
   * @param allMultiTagRobotPoses Combined list of robot poses from multi-tag detection
   * @param allMultiTagRobotPosesAccepted Combined list of accepted multi-tag robot poses
   * @param allMultiTagRobotPosesRejected Combined list of rejected multi-tag robot poses
   * @param allSingleTagPoses Combined list of single-tag poses from all cameras
   * @param allSingleTagRobotPoses Combined list of robot poses from single-tag detection
   * @param allSingleTagRobotPosesAccepted Combined list of accepted single-tag robot poses
   * @param allSingleTagRobotPosesRejected Combined list of rejected single-tag robot poses
   */
  private void logSummaryData(
      List<Pose3d> allMultiTagPoses,
      List<Pose3d> allMultiTagRobotPoses,
      List<Pose3d> allMultiTagRobotPosesAccepted,
      List<Pose3d> allMultiTagRobotPosesRejected,
      List<Pose3d> allSingleTagPoses,
      List<Pose3d> allSingleTagRobotPoses,
      List<Pose3d> allSingleTagRobotPosesAccepted,
      List<Pose3d> allSingleTagRobotPosesRejected) {

    // Log multi-tag summary data
    Logger.recordOutput(
        "Vision/AprilTag/Summary/MultiTag/Poses", allMultiTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/MultiTag/RobotPoses",
        allMultiTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/MultiTag/RobotPosesAccepted",
        allMultiTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/MultiTag/RobotPosesRejected",
        allMultiTagRobotPosesRejected.toArray(new Pose3d[0]));

    // Log single-tag summary data
    Logger.recordOutput(
        "Vision/AprilTag/Summary/SingleTag/Poses", allSingleTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/SingleTag/RobotPoses",
        allSingleTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/SingleTag/RobotPosesAccepted",
        allSingleTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/SingleTag/RobotPosesRejected",
        allSingleTagRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /**
   * Functional interface for components that consume AprilTag vision measurements.
   *
   * <p>Typically implemented by subsystems that handle pose estimation/odometry.
   */
  @FunctionalInterface
  public static interface AprilTagConsumer {
    /**
     * Accepts a vision measurement from the AprilTag subsystem.
     *
     * @param visionRobotPoseMeters Field-relative pose of the robot in meters
     * @param timestampSeconds Timestamp when the measurement was taken, in seconds
     * @param visionMeasurementStdDevs Standard deviations for the measurement (x, y, theta)
     */
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
