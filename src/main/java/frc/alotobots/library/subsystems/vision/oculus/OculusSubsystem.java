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
package frc.alotobots.library.subsystems.vision.oculus;

import static frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants.*;
import static frc.alotobots.library.subsystems.vision.oculus.util.OculusStatus.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIO;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIOInputsAutoLogged;
import frc.alotobots.util.NotificationPresets;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Manages communication and pose estimation with a Meta Quest VR headset.
 *
 * <p>This subsystem leverages the Quest's inside-out SLAM tracking system to provide high-frequency
 * (120Hz) robot pose estimation. Key features:
 *
 * <p>- Global SLAM-based localization - Field mapping and persistence - Sub-centimeter tracking
 * precision - High update rate (120Hz) - Drift-free position tracking - Fast relocalization
 *
 * <p>The system operates in phases: 1. Pre-match mapping to capture field features 2. Initial pose
 * acquisition and alignment 3. Continuous pose updates during match 4. Recovery handling if
 * tracking is lost
 */
public class OculusSubsystem extends SubsystemBase {
  /** Hardware communication interface */
  private final OculusIO io;

  /** Consumer for pose updates from the Oculus */
  private final OculusConsumer oculusConsumer;

  /** Logged inputs from Quest hardware */
  private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();

  /**
   * Creates a new OculusSubsystem.
   *
   * <p>Initializes communication with Quest hardware and prepares logging systems. The subsystem
   * starts in an uninitialized state requiring pose calibration.
   *
   * @param oculusConsumer Consumer that receives pose updates from the headset
   * @param io Interface for Quest hardware communication
   */
  public OculusSubsystem(OculusConsumer oculusConsumer, OculusIO io) {
    this.io = io;
    this.oculusConsumer = oculusConsumer;
    Logger.recordOutput("Oculus/status", "Initialized");
  }

  /**
   * Updates subsystem state and processes Quest data.
   *
   * <p>Called periodically by the command scheduler. This method: - Updates hardware inputs -
   * Processes new pose data - Handles state transitions - Manages reset operations - Updates
   * logging
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    // Add to Kalman filter
    processPose();

    // Notify if we are disconnected
    if (!inputs.connected) {
      NotificationPresets.Oculus.sendOculusDisconnectedNotification();
    } else {
      NotificationPresets.Oculus.sendOculusReconnectedNotification();
    }

    // Notify for battery levels
    if (inputs.batteryPercent < BATTERY_CRITICAL_PERCENT) {
      NotificationPresets.Oculus.sendOculusBatteryCriticalNotification();
    } else if (inputs.batteryPercent < BATTERY_LOW_PERCENT) {
      NotificationPresets.Oculus.sendOculusBatteryLowNotification();
    }

    // Notify for tracking status
    if (!inputs.currentlyTracking) {
      NotificationPresets.Oculus.sendOculusTrackingLostNotification(inputs.trackingLostCounter);
    } else {
      NotificationPresets.Oculus.sendOculusTrackingRegainedNotification();
    }
  }

  /**
   * Returns the battery percentage of the connected Quest headset.
   *
   * @return Battery percentage (0-100)
   */
  public double getBatteryPercent() {
    return inputs.batteryPercent;
  }

  /**
   * Returns the timestamp of the most recent pose update.
   *
   * @return Timestamp in seconds
   */
  public double getTimestamp() {
    return inputs.timestamp;
  }

  /**
   * Checks if the Quest headset is currently connected.
   *
   * @return True if connected, false otherwise
   */
  public boolean isConnected() {
    return inputs.connected;
  }

  /**
   * Gets the current robot pose as estimated by the Quest headset. This incorporates all transforms
   * and offsets to convert from headset to robot coordinates.
   *
   * @return Field-relative robot pose
   */
  @AutoLogOutput(key = "Oculus/Pose")
  public Pose2d getPose() {
    return getOculusPose().transformBy(ROBOT_TO_OCULUS.inverse());
  }

  /**
   * Resets the pose tracking system to a specified position. Must be called only when the robot is
   * disabled to avoid interrupting tracking during a match.
   *
   * @param pose The new reference pose
   */
  public void resetPose(Pose2d pose) {
    // Transform the pose to the Oculus coordinate system w/ offset
    Pose2d oculusSidePose = pose.plus(ROBOT_TO_OCULUS);

    Logger.recordOutput(
        "Oculus/Log",
        String.format("Resetting pose to WPILib: %s, Oculus: %s", pose, oculusSidePose));
    NotificationPresets.Oculus.sendOculusPoseResetNotification(pose);

    // Send the request
    io.setPose(oculusSidePose);
  }

  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Processes the current pose data and forwards it to the consumer if connected and properly
   * tracking. This enables integration with pose estimation systems.
   */
  private void processPose() {
    if (inputs.connected && inputs.currentlyTracking) {
      Pose2d pose = getPose();
      double timestamp = getTimestamp();

      // Make sure we are inside the field
      if (pose.getX() < 0.0
          || pose.getX() > aprilTagFieldLayout.getFieldLength()
          || pose.getY() < 0.0
          || pose.getY() > aprilTagFieldLayout.getFieldWidth()) {
        return;
      }

      // Call the consumer with the new pose
      oculusConsumer.accept(
          SwerveDriveSubsystem.VisionSource.OCULUS, pose, timestamp, OCULUS_STD_DEVS);
    }
  }

  /**
   * Combines Oculus position and orientation into a unified Pose2d.
   *
   * @return Raw Pose2d from the headset's perspective
   */
  @AutoLogOutput(key = "Oculus/RawPose")
  private Pose2d getOculusPose() {
    return inputs.pose2d;
  }

  /**
   * Functional interface for components that consume Oculus vision measurements.
   *
   * <p>Typically implemented by subsystems that handle pose estimation/odometry.
   */
  @FunctionalInterface
  public static interface OculusConsumer {
    /**
     * Accepts a vision measurement from the Oculus subsystem.
     *
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
