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
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.localizationfusion.util.PoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIO;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIOInputsAutoLogged;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

public class AprilTagSubsystem extends SubsystemBase implements PoseSource {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private Pose2d latestMultiTagPose = null;
  private Vector<N3> latestMultiTagStdDevs = null;
  private boolean hasValidMultiTagPose = false;
  private boolean isConnected = false;
  private double lastMultiTagPoseTimestamp = 0.0;

  // Single tag poses are tracked separately but not returned by getCurrentPose
  private Pose2d latestSingleTagPose = null;
  private Vector<N3> latestSingleTagStdDevs = null;
  private boolean hasValidSingleTagPose = false;
  private double lastSingleTagPoseTimestamp = 0.0;

  public AprilTagSubsystem(Consumer<Pose3d> singleTagConsumer, AprilTagIO... io) {
    this.io = io;
    this.inputs = new AprilTagIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagIOInputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
              new Alert(
                      "Vision camera " + CAMERA_CONFIGS[i].name() + " is disconnected.",
                      AlertType.kWarning);
    }

    validateConfiguration();
  }

  // PoseSource interface implementation
  @Override
  public boolean isConnected() {
    return isConnected;
  }

  @Override
  public Pose2d getCurrentPose() {
    double timeSinceLastPose = Timer.getTimestamp() - lastMultiTagPoseTimestamp;
    if (timeSinceLastPose > POSE_TIMEOUT) {
      return null; // Return null if pose is stale
    }
    return hasValidMultiTagPose ? latestMultiTagPose : null;
  }

  public Pose2d getSingleTagPose() {
    double timeSinceLastPose = Timer.getTimestamp() - lastSingleTagPoseTimestamp;
    if (timeSinceLastPose > POSE_TIMEOUT) {
      return null; // Return null if pose is stale
    }
    return hasValidSingleTagPose ? latestSingleTagPose : null;
  }

  @Override
  public Matrix<N3, N1> getStdDevs() {
    return latestMultiTagStdDevs;
  }

  @Override
  public String getSourceName() {
    return "AprilTag";
  }

  @Override
  public void periodic() {
    hasValidMultiTagPose = false;
    hasValidSingleTagPose = false;
    isConnected = false;

    // Update inputs and check camera connections
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/AprilTag/Camera" + CAMERA_CONFIGS[i].name(), inputs[i]);

      // If any camera is connected, consider the system connected
      if (inputs[i].connected) {
        isConnected = true;
      }
      disconnectedAlerts[i].set(!inputs[i].connected);
    }

    // Initialize logging values
    List<Pose3d> allMultiTagPoses = new LinkedList<>();
    List<Pose3d> allMultiTagRobotPoses = new LinkedList<>();
    List<Pose3d> allMultiTagRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allMultiTagRobotPosesRejected = new LinkedList<>();

    List<Pose3d> allSingleTagPoses = new LinkedList<>();
    List<Pose3d> allSingleTagRobotPoses = new LinkedList<>();
    List<Pose3d> allSingleTagRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allSingleTagRobotPosesRejected = new LinkedList<>();

    // Process camera data
    processCameraData(
            allMultiTagPoses,
            allMultiTagRobotPoses,
            allMultiTagRobotPosesAccepted,
            allMultiTagRobotPosesRejected,
            allSingleTagPoses,
            allSingleTagRobotPoses,
            allSingleTagRobotPosesAccepted,
            allSingleTagRobotPosesRejected);

    // Log summary data
    logSummaryData(
            allMultiTagPoses,
            allMultiTagRobotPoses,
            allMultiTagRobotPosesAccepted,
            allMultiTagRobotPosesRejected,
            allSingleTagPoses,
            allSingleTagRobotPoses,
            allSingleTagRobotPosesAccepted,
            allSingleTagRobotPosesRejected);

    // Log pose staleness
    double timeSinceLastMultiTagPose = Timer.getTimestamp() - lastMultiTagPoseTimestamp;
    Logger.recordOutput("Vision/AprilTag/MultiTag/TimeSinceLastPose", timeSinceLastMultiTagPose);
    Logger.recordOutput("Vision/AprilTag/MultiTag/PoseStale", timeSinceLastMultiTagPose > POSE_TIMEOUT);

    double timeSinceLastSingleTagPose = Timer.getTimestamp() - lastSingleTagPoseTimestamp;
    Logger.recordOutput("Vision/AprilTag/SingleTag/TimeSinceLastPose", timeSinceLastSingleTagPose);
    Logger.recordOutput("Vision/AprilTag/SingleTag/PoseStale", timeSinceLastSingleTagPose > POSE_TIMEOUT);
  }

  private void validateConfiguration() {
    for (double factor : CAMERA_STD_DEV_FACTORS) {
      if (factor < 1.0) {
        throw new IllegalArgumentException(
                "[AprilTagSubsystem] STD factor must be >= 1.0, but was: " + factor);
      }
    }
  }

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
              cameraIndex,
              multiTagRobotPoses,
              multiTagRobotPosesAccepted,
              multiTagRobotPosesRejected);

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

  private void processTagPoses(
          int cameraIndex,
          List<Pose3d> multiTagPoses,
          List<Pose3d> singleTagPoses) {

    // Process multi-tag IDs
    for (int tagId : inputs[cameraIndex].multiTagIds) {
      var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
      tagPose.ifPresent(multiTagPoses::add);
    }

    // Process single tag ID if present
    if (inputs[cameraIndex].singleTagId > 0) {
      var tagPose = APRIL_TAG_LAYOUT.getTagPose(inputs[cameraIndex].singleTagId);
      tagPose.ifPresent(singleTagPoses::add);
    }
  }

  private void processMultiTagPoseObservations(
          int cameraIndex,
          List<Pose3d> robotPoses,
          List<Pose3d> robotPosesAccepted,
          List<Pose3d> robotPosesRejected) {

    for (var observation : inputs[cameraIndex].multiTagObservations) {
      boolean rejectPose = shouldRejectMultiTagPose(observation);

      robotPoses.add(observation.pose());
      if (rejectPose) {
        robotPosesRejected.add(observation.pose());
        continue;
      }

      robotPosesAccepted.add(observation.pose());
      updateLatestPoseFromMultiTag(observation, cameraIndex);
      hasValidMultiTagPose = true;
      lastMultiTagPoseTimestamp = Timer.getTimestamp();
    }
  }

  private void processSingleTagPoseObservations(
          int cameraIndex,
          List<Pose3d> robotPoses,
          List<Pose3d> robotPosesAccepted,
          List<Pose3d> robotPosesRejected) {

    for (var observation : inputs[cameraIndex].singleTagObservations) {
      boolean rejectPose = shouldRejectSingleTagPose(observation);

      Pose3d singleTagPose = new Pose3d(
              observation.pose().getX(),
              observation.pose().getY(),
              0.0,
              new Rotation3d(observation.pose().getRotation()));

      robotPoses.add(singleTagPose);

      if (rejectPose) {
        robotPosesRejected.add(singleTagPose);
        continue;
      }

      robotPosesAccepted.add(singleTagPose);

      // Update single tag pose data, but don't use it for getCurrentPose()
      updateLatestPoseFromSingleTag(observation, cameraIndex);
      hasValidSingleTagPose = true;
      lastSingleTagPoseTimestamp = Timer.getTimestamp();

      // Log that we have single tag data (but it's not being used for getCurrentPose)
      Logger.recordOutput("Vision/AprilTag/SingleTag/Available", true);
    }
  }

  private boolean shouldRejectMultiTagPose(AprilTagIO.MultiTagObservation observation) {
    return observation.tagCount() == 0
            || (observation.tagCount() == 1 && observation.ambiguity() > MULTITAG_MAX_AMBIGUITY)
            || Math.abs(observation.pose().getZ()) > MAX_Z_ERROR
            || observation.pose().getX() < 0.0
            || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();
  }

  private boolean shouldRejectSingleTagPose(AprilTagIO.SingleTagObservation observation) {
    return observation.ambiguity() > SINGLE_TAG_MAX_AMBIGUITY
            || observation.tagDistance() > SINGLE_TAG_MAX_DISTANCE
            || observation.pose().getX() < 0.0
            || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();
  }

  private void updateLatestPoseFromMultiTag(AprilTagIO.MultiTagObservation observation, int cameraIndex) {
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = LINEAR_STD_DEV_BASE * stdDevFactor;
    double angularStdDev = ANGULAR_STD_DEV_BASE * stdDevFactor;

    if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
      linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
      angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
    }

    latestMultiTagPose = observation.pose().toPose2d();
    latestMultiTagStdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  private void updateLatestPoseFromSingleTag(AprilTagIO.SingleTagObservation observation, int cameraIndex) {
    // Higher uncertainty for single tag observations - increases with distance
    double stdDevFactor = Math.pow(observation.tagDistance(), 2.0) * SINGLE_TAG_STD_DEV_FACTOR;
    double linearStdDev = LINEAR_STD_DEV_BASE * stdDevFactor;
    double angularStdDev = ANGULAR_STD_DEV_BASE * stdDevFactor;

    if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
      linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
      angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
    }

    latestSingleTagPose = observation.pose();
    latestSingleTagStdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

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

    String cameraPrefix = "Vision/AprilTag/Camera" + CAMERA_CONFIGS[cameraIndex].name();

    // Log only split data (no backward compatibility)
    Logger.recordOutput(cameraPrefix + "/MultiTag/Poses", multiTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(cameraPrefix + "/MultiTag/RobotPoses", multiTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
            cameraPrefix + "/MultiTag/RobotPosesAccepted", multiTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
            cameraPrefix + "/MultiTag/RobotPosesRejected", multiTagRobotPosesRejected.toArray(new Pose3d[0]));

    Logger.recordOutput(cameraPrefix + "/SingleTag/Poses", singleTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(cameraPrefix + "/SingleTag/RobotPoses", singleTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
            cameraPrefix + "/SingleTag/RobotPosesAccepted", singleTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
            cameraPrefix + "/SingleTag/RobotPosesRejected", singleTagRobotPosesRejected.toArray(new Pose3d[0]));
  }

  private void logSummaryData(
          List<Pose3d> allMultiTagPoses,
          List<Pose3d> allMultiTagRobotPoses,
          List<Pose3d> allMultiTagRobotPosesAccepted,
          List<Pose3d> allMultiTagRobotPosesRejected,
          List<Pose3d> allSingleTagPoses,
          List<Pose3d> allSingleTagRobotPoses,
          List<Pose3d> allSingleTagRobotPosesAccepted,
          List<Pose3d> allSingleTagRobotPosesRejected) {

    Logger.recordOutput("Vision/AprilTag/Summary/MultiTag/Poses", allMultiTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AprilTag/Summary/MultiTag/RobotPoses", allMultiTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
            "Vision/AprilTag/Summary/MultiTag/RobotPosesAccepted", allMultiTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
            "Vision/AprilTag/Summary/MultiTag/RobotPosesRejected", allMultiTagRobotPosesRejected.toArray(new Pose3d[0]));

    Logger.recordOutput("Vision/AprilTag/Summary/SingleTag/Poses", allSingleTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AprilTag/Summary/SingleTag/RobotPoses", allSingleTagRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
            "Vision/AprilTag/Summary/SingleTag/RobotPosesAccepted", allSingleTagRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
            "Vision/AprilTag/Summary/SingleTag/RobotPosesRejected", allSingleTagRobotPosesRejected.toArray(new Pose3d[0]));
  }
}