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
package frc.alotobots.library.subsystems.vision.oculus.io;

import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants.OCULUS_CONNECTION_TIMEOUT;
import static frc.alotobots.library.subsystems.vision.oculus.util.OculusStatus.Miso.*;
import static frc.alotobots.library.subsystems.vision.oculus.util.OculusStatus.Mosi.*;
import static frc.alotobots.util.NotificationPresets.Oculus.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import frc.alotobots.library.subsystems.vision.oculus.util.QuestCommandRetryHandler;
import org.littletonrobotics.junction.Logger;

/** Implementation of OculusIO for real hardware communication via NetworkTables. */
public class OculusIOReal implements OculusIO {
  /** NetworkTable for Oculus communication */
  private final NetworkTable nt4Table;

  /** Subscriber for MISO (Master In Slave Out) values */
  private final IntegerSubscriber questMiso;

  /** Publisher for MOSI (Master Out Slave In) values */
  private final IntegerPublisher questMosi;

  /** Subscriber for frame count updates */
  private final IntegerSubscriber questFrameCount;

  /** Subscriber for timestamp updates */
  private final DoubleSubscriber questTimestamp;

  /** Subscriber for position updates */
  private final FloatArraySubscriber questPosition;

  /** Subscriber for quaternion orientation updates */
  private final FloatArraySubscriber questQuaternion;

  /** Subscriber for Euler angle updates */
  private final FloatArraySubscriber questEulerAngles;

  /** Subscriber for battery percentage updates */
  private final DoubleSubscriber questBatteryPercent;

  /** Subscriber for tracking status */
  private final BooleanSubscriber questTrackingStatus;

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub;

  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub;

  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  /** Publisher for pose reset commands */
  private final DoubleArrayPublisher resetPosePub;

  /** Pose transform tracking for robot code side updates */
  private Pose2d resetPosition = new Pose2d();

  /** Command retry handler for Quest commands */
  private final QuestCommandRetryHandler retryHandler;

  /** Attempts to run a command with success and failure callbacks. */
  private void tryCommandUntilOk(
      int maxAttempts, int command, int expectedResponse, Runnable onSuccess, Runnable onFailure) {
    retryHandler.startRetry(
        maxAttempts, command, expectedResponse, COMMAND_CLEAR, onSuccess, onFailure);
  }

  /**
   * Creates a new OculusIOReal instance and initializes all NetworkTable publishers and
   * subscribers.
   */
  public OculusIOReal() {
    nt4Table = NetworkTableInstance.getDefault().getTable("questnav");
    questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    questMosi = nt4Table.getIntegerTopic("mosi").publish();
    questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(-1);
    questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(-1.0);
    questPosition =
        nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questQuaternion =
        nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    questEulerAngles =
        nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questBatteryPercent = nt4Table.getDoubleTopic("device/batteryPercent").subscribe(-1.0);
    questTrackingStatus = nt4Table.getBooleanTopic("device/isTracking").subscribe(false);
    heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
    heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
    resetPosePub = nt4Table.getDoubleArrayTopic("resetpose").publish();

    // Initialize the retry handler
    retryHandler = new QuestCommandRetryHandler(questMosi, 0.05, 0.2);
  }

  @Override
  public void updateInputs(OculusIOInputs inputs) {
    Logger.recordOutput(
        "Oculus/Debug",
        String.format(
            "timestamp %f, hbReqSub, %f", Timer.getTimestamp(), heartbeatRequestSub.get()));
    inputs.connected =
        Seconds.of(Timer.getTimestamp() - heartbeatRequestSub.getLastChange())
            .lt(OCULUS_CONNECTION_TIMEOUT);
    inputs.position = questPosition.get();
    inputs.quaternion = questQuaternion.get();
    inputs.eulerAngles = questEulerAngles.get();
    inputs.timestamp = questTimestamp.getAtomic().serverTime;
    inputs.frameCount = (int) questFrameCount.get();
    inputs.batteryPercent = questBatteryPercent.get();
    inputs.isTracking = questTrackingStatus.get();
    inputs.misoValue = (int) questMiso.get();

    // Update the retry handler
    retryHandler.update((int) questMiso.get(), COMMAND_CLEAR);

    processHeartbeat();
  }

  @Override
  public void resetPose(Pose2d oculusTargetPose) {
    resetPosePub.set(
        new double[] {
          oculusTargetPose.getX(),
          oculusTargetPose.getY(),
          oculusTargetPose.getRotation().getDegrees()
        });

    // Using callbacks to handle success and failure
    tryCommandUntilOk(
        5,
        COMMAND_RESET_POSE,
        STATUS_POSE_RESET_COMPLETE,
        () -> {
          Logger.recordOutput("Oculus/Log", "Pose reset successful");
          sendOculusPoseResetNotification(oculusTargetPose);
          cleanupResponses();
        },
        () -> {
          Logger.recordOutput("Oculus/Log", "Pose reset failed");
          sendOculusPoseResetFailedNotification();
        });
  }

  @Override
  public void resetHeading() {
    // Using callbacks to handle success and failure
    tryCommandUntilOk(
        5,
        COMMAND_RESET_HEADING,
        STATUS_HEADING_RESET_COMPLETE,
        () -> {
          Logger.recordOutput("Oculus/Log", "Heading reset successful");
          sendOculusHeadingResetNotification();
          cleanupResponses();
        },
        () -> {
          Logger.recordOutput("Oculus/Log", "Heading reset failed");
          sendOculusHeadingResetFailedNotification();
        });
  }

  private void cleanupResponses() {
    if (questMiso.get() != STATUS_READY) {
      switch ((int) questMiso.get()) {
        case STATUS_POSE_RESET_COMPLETE -> {
          Logger.recordOutput("Oculus/Log", "Pose reset complete");
          questMosi.set(COMMAND_CLEAR);
        }
        case STATUS_HEADING_RESET_COMPLETE -> {
          Logger.recordOutput("Oculus/Log", "Heading reset complete");
          questMosi.set(COMMAND_CLEAR);
        }
        case STATUS_PING_RESPONSE -> {
          Logger.recordOutput("Oculus/Log", "Ping response received");
          questMosi.set(COMMAND_CLEAR);
        }
      }
    }
  }

  /** Process heartbeat requests from Quest and respond with the same ID */
  private void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();

    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      // Echo back the same ID as response
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }
}
