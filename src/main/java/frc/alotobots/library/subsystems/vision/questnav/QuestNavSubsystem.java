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
package frc.alotobots.library.subsystems.vision.questnav;

import static frc.alotobots.library.subsystems.vision.questnav.constants.QuestNavConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.vision.questnav.io.QuestNavIO;
import frc.alotobots.library.subsystems.vision.questnav.io.QuestNavIOInputsAutoLogged;
import frc.alotobots.util.NotificationPresets;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * QuestNavSubsystem handles robot localization using a Meta Quest VR headset.
 *
 * <p>This subsystem leverages the Quest's inside-out SLAM tracking system to provide high-frequency
 * (120Hz) robot pose estimation with sub-centimeter precision. The Quest's visual-inertial odometry
 * provides drift-free position tracking through persistent field mapping.
 */
public class QuestNavSubsystem extends SubsystemBase {

  // IO and inputs
  private final QuestNavIO io;
  private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

  // Tracking for pose validity and timing
  private double lastQuestNavPoseTimestamp = -1;

  // Consumer for applying vision measurements to odometry/pose estimation
  private final QuestNavConsumer questNavConsumer;

  /**
   * Creates a new QuestNavSubsystem with the given consumer and IO interface.
   *
   * @param questNavConsumer Consumer that will receive vision measurement updates
   * @param io QuestNavIO object representing the Quest hardware interface
   */
  public QuestNavSubsystem(QuestNavConsumer questNavConsumer, QuestNavIO io) {
    this.io = io;
    this.questNavConsumer = questNavConsumer;
    Logger.recordOutput("QuestNav/status", "Initialized");
  }

  /**
   * Periodic update method - called repeatedly by the command scheduler.
   *
   * <p>This method handles:
   *
   * <ul>
   *   <li>Reading Quest hardware data
   *   <li>Processing pose observations to estimate robot position
   *   <li>Filtering out invalid or unreliable detections
   *   <li>Providing position updates to the QuestNavConsumer
   *   <li>Monitoring connection, battery, and tracking status
   *   <li>Logging data for debugging and analysis
   * </ul>
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/QuestNav", inputs);

    // Alert if we are disconnected, low battery, not tracking, etc
    processAlerts();

    // Initialize logging value lists
    List<Pose3d> questNavRobotPoses = new LinkedList<>();
    List<Pose3d> questNavRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> questNavRobotPosesRejected = new LinkedList<>();

    // Process observations
    processQuestNavPoseObservations(
        questNavRobotPoses, questNavRobotPosesAccepted, questNavRobotPosesRejected);

    // Log data
    logQuestNavData(questNavRobotPoses, questNavRobotPosesAccepted, questNavRobotPosesRejected);

    // Log time since last pose
    Logger.recordOutput(
        "Vision/QuestNav/TimeSinceLastPose", Timer.getTimestamp() - lastQuestNavPoseTimestamp);
  }

  /**
   * Processes Quest pose observations, filtering valid measurements and updating pose estimates.
   *
   * @param questNavRobotPoses List to populate with all Quest robot poses
   * @param questNavRobotPosesAccepted List to populate with accepted Quest robot poses
   * @param questNavRobotPosesRejected List to populate with rejected Quest robot poses
   */
  private void processQuestNavPoseObservations(
      List<Pose3d> questNavRobotPoses,
      List<Pose3d> questNavRobotPosesAccepted,
      List<Pose3d> questNavRobotPosesRejected) {

    for (var observation : inputs.questNavObservations) {
      // Check if pose should be rejected based on criteria
      boolean rejectPose = shouldRejectQuestNavPose(observation);

      // Add to all poses list
      questNavRobotPoses.add(observation.pose());

      // If rejected, add to rejected list and skip processing
      if (rejectPose) {
        questNavRobotPosesRejected.add(observation.pose());
        continue;
      }

      // Add to accepted list
      questNavRobotPosesAccepted.add(observation.pose());

      // Update pose estimation with this observation
      updatePoseFromQuestNav(observation);

      lastQuestNavPoseTimestamp = observation.timestamp();
    }
  }

  /**
   * Logs Quest pose data for debugging and analysis.
   *
   * @param questNavRobotPoses List of all Quest robot poses
   * @param questNavRobotPosesAccepted List of accepted Quest robot poses
   * @param questNavRobotPosesRejected List of rejected Quest robot poses
   */
  private void logQuestNavData(
      List<Pose3d> questNavRobotPoses,
      List<Pose3d> questNavRobotPosesAccepted,
      List<Pose3d> questNavRobotPosesRejected) {

    Logger.recordOutput("Vision/QuestNav/RobotPoses", questNavRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/QuestNav/RobotPosesAccepted", questNavRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/QuestNav/RobotPosesRejected", questNavRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /**
   * Updates the robot's pose estimation based on a Quest observation.
   *
   * @param observation The Quest observation to use for the update
   */
  private void updatePoseFromQuestNav(QuestNavIO.QuestNavObservation observation) {

    // TODO: dynamic STD calculation based on latency?
    questNavConsumer.accept(
        SwerveDriveSubsystem.VisionSource.QUESTNAV,
        observation.pose().toPose2d(),
        observation.timestamp(),
        QUESTNAV_STD_DEVS);
  }

  /**
   * Determines if a Quest pose observation should be rejected.
   *
   * @param observation The Quest observation to evaluate
   * @return True if the pose should be rejected, false otherwise
   */
  private boolean shouldRejectQuestNavPose(QuestNavIO.QuestNavObservation observation) {
    return observation.pose().getX() < 0.0
        || observation.pose().getX() > QUESTNAV_FIELD_LAYOUT.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > QUESTNAV_FIELD_LAYOUT.getFieldWidth();
  }

  /**
   * Processes and sends alerts for Quest hardware status.
   *
   * <p>Monitors connection status, battery levels, and tracking state, sending appropriate
   * notifications when issues are detected or resolved.
   */
  private void processAlerts() {
    // Notify if we are disconnected
    if (!inputs.connected) {
      NotificationPresets.QuestNav.sendQuestNavDisconnectedNotification();
    } else {
      NotificationPresets.QuestNav.sendQuestNavReconnectedNotification();
    }

    // Notify for battery levels
    if (inputs.batteryPercent < BATTERY_CRITICAL_PERCENT) {
      NotificationPresets.QuestNav.sendQuestNavBatteryCriticalNotification();
    } else if (inputs.batteryPercent < BATTERY_LOW_PERCENT) {
      NotificationPresets.QuestNav.sendQuestNavBatteryLowNotification();
    }

    // Notify for tracking status
    if (!inputs.currentlyTracking) {
      NotificationPresets.QuestNav.sendQuestNavTrackingLostNotification(inputs.trackingLostCounter);
    } else {
      NotificationPresets.QuestNav.sendQuestNavTrackingRegainedNotification();
    }
  }

  /**
   * Resets the pose tracking system to a specified position.
   *
   * @param pose The new reference pose
   */
  public void resetPose(Pose2d pose) {
    Pose3d questFieldRelative = new Pose3d(pose).plus(ROBOT_TO_QUEST);

    NotificationPresets.QuestNav.sendQuestNavPoseResetNotification(questFieldRelative);

    // Send the request
    io.setPose(questFieldRelative);
  }

  /**
   * Resets the pose tracking system to a specified position.
   *
   * @param pose The new reference pose
   */
  public void resetPose(Pose3d pose) {
    Pose3d questFieldRelative = pose.plus(ROBOT_TO_QUEST);

    NotificationPresets.QuestNav.sendQuestNavPoseResetNotification(questFieldRelative);

    // Send the request
    io.setPose(questFieldRelative);
  }

  /**
   * Functional interface for components that consume QuestNav vision measurements.
   *
   * <p>Typically implemented by subsystems that handle pose estimation/odometry.
   */
  @FunctionalInterface
  public static interface QuestNavConsumer {
    /**
     * Accepts a vision measurement from the QuestNav subsystem.
     *
     * @param source Vision source identifier
     * @param visionRobotPoseMeters Field-relative pose of the robot in meters
     * @param timestampSeconds Timestamp when the measurement was taken, in seconds
     * @param visionMeasurementStdDevs Standard deviations for the measurement (x, y, theta)
     */
    public void accept(
        SwerveDriveSubsystem.VisionSource source,
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
