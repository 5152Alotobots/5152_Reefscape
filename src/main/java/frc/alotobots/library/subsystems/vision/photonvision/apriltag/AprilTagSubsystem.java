/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.localizationfusion.PoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIO;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.AprilTagIOInputsAutoLogged;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AprilTagSubsystem extends SubsystemBase implements PoseSource {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private Pose2d latestPose = null;
  private Vector<N3> latestStdDevs = null;
  private boolean hasValidPose = false;
  private boolean isConnected = false;
  private double lastPoseTimestamp = 0.0;

  public AprilTagSubsystem(AprilTagIO... io) {
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
    double timeSinceLastPose = Timer.getTimestamp() - lastPoseTimestamp;
    if (timeSinceLastPose > POSE_TIMEOUT) {
      return null; // Return null if pose is stale
    }
    return hasValidPose ? latestPose : null;
  }

  @Override
  public Matrix<N3, N1> getStdDevs() {
    return latestStdDevs;
  }

  @Override
  public String getSourceName() {
    return "AprilTag";
  }

  @Override
  public void periodic() {
    hasValidPose = false;
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
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Process camera data
    processCameraData(allTagPoses, allRobotPoses, allRobotPosesAccepted, allRobotPosesRejected);

    // Log summary data
    logSummaryData(allTagPoses, allRobotPoses, allRobotPosesAccepted, allRobotPosesRejected);

    // Log pose staleness
    double timeSinceLastPose = Timer.getTimestamp() - lastPoseTimestamp;
    Logger.recordOutput("Vision/AprilTag/TimeSinceLastPose", timeSinceLastPose);
    Logger.recordOutput("Vision/AprilTag/PoseStale", timeSinceLastPose > POSE_TIMEOUT);
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
      List<Pose3d> allTagPoses,
      List<Pose3d> allRobotPoses,
      List<Pose3d> allRobotPosesAccepted,
      List<Pose3d> allRobotPosesRejected) {

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      processTagPoses(cameraIndex, tagPoses);
      processPoseObservations(cameraIndex, robotPoses, robotPosesAccepted, robotPosesRejected);

      logCameraData(cameraIndex, tagPoses, robotPoses, robotPosesAccepted, robotPosesRejected);

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }
  }

  private void processTagPoses(int cameraIndex, List<Pose3d> tagPoses) {
    for (int tagId : inputs[cameraIndex].tagIds) {
      var tagPose = APRIL_TAG_LAYOUT.getTagPose(tagId);
      tagPose.ifPresent(tagPoses::add);
    }
  }

  private void processPoseObservations(
      int cameraIndex,
      List<Pose3d> robotPoses,
      List<Pose3d> robotPosesAccepted,
      List<Pose3d> robotPosesRejected) {

    for (var observation : inputs[cameraIndex].poseObservations) {
      boolean rejectPose = shouldRejectPose(observation);

      robotPoses.add(observation.pose());
      if (rejectPose) {
        robotPosesRejected.add(observation.pose());
        continue;
      }

      robotPosesAccepted.add(observation.pose());
      updateLatestPose(observation, cameraIndex);
      hasValidPose = true;
      lastPoseTimestamp = Timer.getTimestamp();
    }
  }

  private boolean shouldRejectPose(AprilTagIO.PoseObservation observation) {
    return observation.tagCount() == 0
        || (observation.tagCount() == 1 && observation.ambiguity() > MAX_AMBIGUITY)
        || Math.abs(observation.pose().getZ()) > MAX_Z_ERROR
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > APRIL_TAG_LAYOUT.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > APRIL_TAG_LAYOUT.getFieldWidth();
  }

  private void updateLatestPose(AprilTagIO.PoseObservation observation, int cameraIndex) {
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = LINEAR_STD_DEV_BASE * stdDevFactor;
    double angularStdDev = ANGULAR_STD_DEV_BASE * stdDevFactor;

    if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
      linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
      angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
    }

    latestPose = observation.pose().toPose2d();
    latestStdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  private void logCameraData(
      int cameraIndex,
      List<Pose3d> tagPoses,
      List<Pose3d> robotPoses,
      List<Pose3d> robotPosesAccepted,
      List<Pose3d> robotPosesRejected) {
    String cameraPrefix = "Vision/AprilTag/Camera" + CAMERA_CONFIGS[cameraIndex].name();
    Logger.recordOutput(cameraPrefix + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(cameraPrefix + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        cameraPrefix + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
  }

  private void logSummaryData(
      List<Pose3d> allTagPoses,
      List<Pose3d> allRobotPoses,
      List<Pose3d> allRobotPosesAccepted,
      List<Pose3d> allRobotPosesRejected) {
    Logger.recordOutput("Vision/AprilTag/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/AprilTag/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/AprilTag/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }
}
