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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIO;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIOInputsAutoLogged;
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

  /** Logged inputs from Quest hardware */
  private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();

  private Transform2d offsetTransform = new Transform2d();

  /**
   * Creates a new OculusSubsystem.
   *
   * <p>Initializes communication with Quest hardware and prepares logging systems. The subsystem
   * starts in an uninitialized state requiring pose calibration.
   *
   * @param io Interface for Quest hardware communication
   */
  public OculusSubsystem(OculusIO io) {
    this.io = io;
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
  }

  public double getBatteryPercent() {
    return inputs.batteryPercent;
  }

  public double getTimestamp() {
    return inputs.timestamp;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  @AutoLogOutput(key = "Oculus/Pose")
  public Pose2d getPose() {
    return getOculusPose().transformBy(ROBOT_TO_OCULUS.inverse()).plus(offsetTransform);
  }

  /**
   * Does a "hard reset" of the oculus pose. This should be called PRIOR to the start of the match
   * to avoid latency associated with resetting the Oculus side pose.
   */
  public void resetPose(Pose2d pose) {
    if (DriverStation.isEnabled()) {
      Logger.recordOutput(
          "Oculus/Log",
          "resetPose() called while the robot is enabled. This shouldn't happen! Ignoring.");
      return;
    }

    if (POSE_RESET_STRATEGY.equals(PoseResetStrategy.ROBOT_SIDE)) {
      // Reset the pose on the Oculus side
      io.resetPose(0, 0, 0);
      // Set the offset transform to the new pose
      updateTransform(pose);
      Logger.recordOutput("Oculus/Log", "Resetting pose to: " + pose);
    } else {
      io.resetPose(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
      Logger.recordOutput("Oculus/Log", "Resetting pose to: " + pose);
    }
  }

  public void updateTransform(Pose2d pose) {
    if (!POSE_RESET_STRATEGY.equals(PoseResetStrategy.ROBOT_SIDE)) {
      Logger.recordOutput(
          "Oculus/Log", "updateTransform() called when not using ROBOT_SIDE. Ignoring.");
      return;
    }
    // Update the offset transform to the new pose
    Logger.recordOutput("Oculus/Log", "Updating offset transform to: " + pose);
    offsetTransform = new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  private Rotation2d getOculusYaw() {
    return Rotation2d.fromDegrees(-inputs.eulerAngles[1]);
  }

  private Translation2d getOculusTranslation() {
    float[] oculusPosition = inputs.position;
    return new Translation2d(oculusPosition[2], -oculusPosition[0]);
  }

  private Pose2d getOculusPose() {
    return new Pose2d(getOculusTranslation(), getOculusYaw());
  }
}
