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

import edu.wpi.first.networktables.*;

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

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub;

  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub;

  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  /** Publisher for pose reset commands */
  private final DoubleArrayPublisher resetPosePub;

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
    questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(-1.0);
    heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
    heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
    resetPosePub = nt4Table.getDoubleArrayTopic("resetpose").publish();
  }

  @Override
  public void updateInputs(OculusIOInputs inputs) {
    inputs.position = questPosition.get();
    inputs.quaternion = questQuaternion.get();
    inputs.eulerAngles = questEulerAngles.get();
    inputs.timestamp = questTimestamp.get();
    inputs.frameCount = (int) questFrameCount.get();
    inputs.batteryPercent = questBatteryPercent.get();
    inputs.misoValue = (int) questMiso.get();
    processHeartbeat();
  }

  @Override
  public void setMosi(int value) {
    questMosi.set(value);
  }

  @Override
  public void setResetPose(double x, double y, double rotation) {
    resetPosePub.set(new double[] {x, y, rotation});
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
