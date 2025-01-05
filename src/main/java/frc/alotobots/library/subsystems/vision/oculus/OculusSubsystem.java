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
package frc.alotobots.library.subsystems.vision.oculus;

import static frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.vision.localizationfusion.PoseSource;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIO;
import frc.alotobots.library.subsystems.vision.oculus.io.OculusIOInputsAutoLogged;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Manages communication and pose estimation with a Meta Quest VR headset.
 *
 * <p>This subsystem leverages the Quest's inside-out SLAM tracking system to provide high-frequency
 * (120Hz) robot pose estimation. Key features:
 *
 * <p>- Global SLAM-based localization - Field mapping and persistence - Sub-millimeter tracking
 * precision - High update rate (120Hz) - Drift-free position tracking - Fast relocalization
 *
 * <p>The system operates in phases: 1. Pre-match mapping to capture field features 2. Initial pose
 * acquisition and alignment 3. Continuous pose updates during match 4. Recovery handling if
 * tracking is lost
 */
public class OculusSubsystem extends SubsystemBase implements PoseSource {
  /** Status indicating system is ready for commands */
  private static final int STATUS_READY = 0;

  /** Status indicating heading reset completion */
  private static final int STATUS_HEADING_RESET_COMPLETE = 99;

  /** Status indicating pose reset completion */
  private static final int STATUS_POSE_RESET_COMPLETE = 98;

  /** Status indicating ping response receipt */
  private static final int STATUS_PING_RESPONSE = 97;

  /** Connection timeout threshold (seconds) */
  private static final double CONNECTION_TIMEOUT = 1;

  /** Hardware communication interface */
  private final OculusIO io;

  /** Logged inputs from Quest hardware */
  private final OculusIOInputsAutoLogged inputs = new OculusIOInputsAutoLogged();

  /** Flag indicating active pose reset */
  @Getter private boolean poseResetInProgress = false;

  /** Flag indicating active heading reset */
  @Getter private boolean headingResetInProgress = false;

  /** Flag indicating active ping operation */
  @Getter private boolean pingInProgress = false;

  /** Previous connection state */
  @Getter private boolean wasConnected = false;

  /** Reset operation start timestamp */
  private double resetStartTime = 0;

  /** Current reset attempt counter */
  private int currentResetAttempt = 0;

  /** Previous Quest timestamp */
  private double lastTimestamp = 0.0;

  /** Current Quest timestamp */
  private double currentTimestamp = 0.0;

  /** Target pose for reset operation */
  private Pose2d pendingResetPose = null;

  /** Current robot pose estimate */
  private Pose2d currentPose = null;

  private double lastQuestUpdateTime = Timer.getTimestamp();

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

    lastTimestamp = currentTimestamp;
    currentTimestamp = inputs.timestamp;

    // Update current pose
    var oculusPose = getOculusPose();
    currentPose = oculusPose.transformBy(ROBOT_TO_OCULUS.inverse());
    Logger.recordOutput("Oculus/status/poses/headsetPose", oculusPose);
    Logger.recordOutput("Oculus/status/poses/robotPose", currentPose);

    handleResetTimeout();
    handleResetCompletion();
    handlePingResponse();
  }

  /**
   * Manages timeouts for reset operations.
   *
   * <p>Reset operations that exceed the timeout threshold are either: - Retried up to the maximum
   * attempt limit - Abandoned with appropriate status logging
   */
  private void handleResetTimeout() {
    double currentTime = Timer.getTimestamp();
    if (currentTime - resetStartTime > RESET_TIMEOUT_SECONDS) {
      if (headingResetInProgress) handleReset(true);
      if (poseResetInProgress) handleReset(false);
    }
  }

  /**
   * Processes reset operation outcomes.
   *
   * @param isHeadingReset true for heading reset, false for pose reset
   */
  private void handleReset(boolean isHeadingReset) {
    if (currentResetAttempt < MAX_RESET_ATTEMPTS) {
      String resetType = isHeadingReset ? "Heading" : "Pose";
      Logger.recordOutput(
          "Oculus/status",
          resetType + " Reset attempt " + (currentResetAttempt + 1) + " timed out, retrying...");
      currentResetAttempt++;
      resetStartTime = Timer.getTimestamp();

      io.setMosi(0); // Clear command

      if (isHeadingReset) {
        io.setMosi(1); // Request heading reset
      } else {
        io.setResetPose(
            pendingResetPose.getX(),
            pendingResetPose.getY(),
            pendingResetPose.getRotation().getDegrees());
        io.setMosi(2); // Request pose reset
      }
    } else {
      Logger.recordOutput(
          "Oculus/status",
          (isHeadingReset ? "Heading" : "Pose")
              + " Reset failed after "
              + MAX_RESET_ATTEMPTS
              + " attempts");
      if (isHeadingReset) {
        clearHeadingResetState();
      } else {
        clearPoseResetState();
      }
    }
  }

  /**
   * Handles completion of reset operations.
   *
   * <p>Processes successful reset confirmations from Quest and updates system state.
   */
  private void handleResetCompletion() {
    if (headingResetInProgress && inputs.misoValue == STATUS_HEADING_RESET_COMPLETE) {
      Logger.recordOutput(
          "Oculus/status",
          "Heading Reset completed successfully on attempt " + (currentResetAttempt + 1));
      clearHeadingResetState();
    }
    if (poseResetInProgress && inputs.misoValue == STATUS_POSE_RESET_COMPLETE) {
      Logger.recordOutput(
          "Oculus/status",
          "Pose Reset completed successfully on attempt " + (currentResetAttempt + 1));
      clearPoseResetState();
    }
  }

  /**
   * Handles ping response from Quest.
   *
   * <p>Processes communication test responses and updates status.
   */
  private void handlePingResponse() {
    if (inputs.misoValue == STATUS_PING_RESPONSE) {
      Logger.recordOutput("Oculus/status", "Ping response received");
      io.setMosi(0); // Clear command
      pingInProgress = false;
    }
  }

  /**
   * Clears pose reset operation state.
   *
   * <p>Resets all state variables related to pose reset operations.
   */
  private void clearPoseResetState() {
    Logger.recordOutput("Oculus/status", "Clearing pose reset state");
    poseResetInProgress = false;
    pendingResetPose = null;
    currentResetAttempt = 0;
    io.setMosi(0); // Clear command
  }

  /**
   * Clears heading reset operation state.
   *
   * <p>Resets all state variables related to heading reset operations.
   */
  private void clearHeadingResetState() {
    Logger.recordOutput("Oculus/status", "Clearing heading reset state");
    headingResetInProgress = false;
    currentResetAttempt = 0;
    io.setMosi(0); // Clear command
  }

  /**
   * Gets current Quest rotation in field frame.
   *
   * <p>Converts Quest IMU data to field-relative rotation.
   *
   * @return Current headset rotation
   */
  private Rotation2d getOculusYaw() {
    return Rotation2d.fromDegrees(-inputs.eulerAngles[1]);
  }

  /**
   * Gets current Quest position in field frame.
   *
   * <p>Converts Quest SLAM position to field coordinates.
   *
   * @return Current headset position
   */
  private Translation2d getOculusPosition() {
    return new Translation2d(inputs.position[2], -inputs.position[0]);
  }

  /**
   * Gets complete Quest pose in field frame.
   *
   * <p>Combines position and rotation data into a field-relative pose.
   *
   * @return Current headset pose
   */
  private Pose2d getOculusPose() {
    return new Pose2d(getOculusPosition(), getOculusYaw());
  }

  /**
   * Initiates pose reset operation.
   *
   * <p>Attempts to reset Quest SLAM origin to align with target pose.
   *
   * @param targetPose Desired robot pose after reset
   * @return true if reset initiated successfully
   */
  public boolean resetToPose(Pose2d targetPose) {
    if (poseResetInProgress) {
      Logger.recordOutput("Oculus/status", "Cannot reset pose - reset already in progress");
      return false;
    }

    if (inputs.misoValue != STATUS_READY) {
      Logger.recordOutput(
          "Oculus/status", "Cannot reset pose - Quest busy (MISO=" + inputs.misoValue + ")");
      return false;
    }

    targetPose = targetPose.plus(ROBOT_TO_OCULUS);
    pendingResetPose = targetPose;
    Logger.recordOutput(
        "Oculus/status",
        String.format(
            "Initiating pose reset to X:%.2f Y:%.2f Rot:%.2f°",
            targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()));

    io.setResetPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees());
    poseResetInProgress = true;
    resetStartTime = Timer.getTimestamp();
    currentResetAttempt = 0;
    io.setMosi(2); // Request pose reset

    return true;
  }

  /**
   * Initiates heading reset operation.
   *
   * <p>Attempts to zero current Quest heading measurement.
   *
   * @return true if reset initiated successfully
   */
  public boolean zeroHeading() {
    if (headingResetInProgress) {
      Logger.recordOutput("Oculus/status", "Cannot zero heading - reset already in progress");
      return false;
    }

    if (inputs.misoValue != STATUS_READY) {
      Logger.recordOutput(
          "Oculus/status", "Cannot zero heading - Quest busy (MISO=" + inputs.misoValue + ")");
      return false;
    }

    Logger.recordOutput("Oculus/status", "Zeroing heading");
    headingResetInProgress = true;
    resetStartTime = Timer.getTimestamp();
    currentResetAttempt = 0;
    io.setMosi(1); // Request heading reset
    return true;
  }

  /**
   * Tests Quest communication.
   *
   * <p>Sends ping command to verify hardware connectivity.
   *
   * @return true if ping initiated successfully
   */
  public boolean ping() {
    if (pingInProgress) {
      Logger.recordOutput("Oculus/status", "Cannot ping - ping already in progress");
      return false;
    }

    if (inputs.misoValue != STATUS_READY) {
      Logger.recordOutput(
          "Oculus/status", "Cannot ping - system busy (MISO=" + inputs.misoValue + ")");
      return false;
    }

    Logger.recordOutput("Oculus/status", "Sending ping...");
    Logger.recordOutput("Oculus/ping/sendTime", Timer.getTimestamp());
    pingInProgress = true;
    io.setMosi(3); // Send ping
    return true;
  }

  // PoseSource interface implementation

  /**
   * {@inheritDoc}
   *
   * <p>Quest connection is determined by checking if we've received any new Quest timestamp updates
   * within our timeout window. Small variations in update timing are allowed, only triggering
   * disconnect on significant delays.
   */
  @Override
  public boolean isConnected() {
    // If timestamp has changed since last check, update our last update time
    if (inputs.timestamp != lastTimestamp) {
      lastQuestUpdateTime = Timer.getTimestamp();
    }

    // Only consider disconnected if we haven't seen ANY new timestamps
    // for longer than our timeout period
    return (Timer.getTimestamp() - lastQuestUpdateTime) < CONNECTION_TIMEOUT;
  }

  /**
   * {@inheritDoc}
   *
   * <p>Returns field-relative robot pose from Quest SLAM.
   */
  @Override
  public Pose2d getCurrentPose() {
    return currentPose;
  }

  /**
   * {@inheritDoc}
   *
   * <p>Returns constant measurement uncertainty for Quest tracking.
   */
  @Override
  public Matrix<N3, N1> getStdDevs() {
    return OCULUS_STD_DEVS;
  }

  /** {@inheritDoc} */
  @Override
  public String getSourceName() {
    return "Oculus Quest";
  }
}
