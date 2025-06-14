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

import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

/** Implementation of OculusIO for real hardware communication via QuestNav vendor library. */
public class OculusIOReal implements OculusIO {
  /** QuestNav instance for communication with the Quest headset */
  private final QuestNav questNav;

  /** Creates a new OculusIOReal instance using the QuestNav vendor library. */
  public OculusIOReal() {
    questNav = new QuestNav();
  }

  @Override
  public void updateInputs(OculusIOInputs inputs) {
    // Update connection status
    inputs.connected = questNav.isConnected();

    // Update frame data
    inputs.frameCount = questNav.getFrameCount();
    inputs.timestamp = questNav.getTimestamp();
    inputs.pose2d = questNav.getPose();

    // Update device data
    inputs.batteryPercent = questNav.getBatteryPercent();
    inputs.currentlyTracking = questNav.isTracking();
    inputs.trackingLostCounter = questNav.getTrackingLostCounter();

    // Other data
    inputs.latency = questNav.getLatency();

    // Process any pending command responses
    questNav.commandPeriodic();
  }

  @Override
  public void setPose(Pose2d oculusTargetPose) {
    questNav.setPose(oculusTargetPose);
    Logger.recordOutput(
        "Oculus/Log",
        String.format(
            "Pose reset requested to: (%.2f, %.2f, %.2f°)",
            oculusTargetPose.getX(),
            oculusTargetPose.getY(),
            oculusTargetPose.getRotation().getDegrees()));
  }
}
