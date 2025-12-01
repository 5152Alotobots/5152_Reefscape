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
package frc.alotobots.library.subsystems.vision.questnav.io;

import static edu.wpi.first.units.Units.Degrees;
import static frc.alotobots.library.subsystems.vision.questnav.constants.QuestNavConstants.ROBOT_TO_QUEST;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Implementation of QuestNavIO for real hardware communication via QuestNav vendor library. */
public class QuestNavIOReal implements QuestNavIO {
  /** QuestNav instance for communication with the Quest headset */
  private final QuestNav questNav;

  /** Creates a new QuestNavIOReal instance using the QuestNav vendor library. */
  public QuestNavIOReal() {
    questNav = new QuestNav();
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    // Run periodic loop for QuestNav
    questNav.commandPeriodic();

    inputs.connected = questNav.isConnected();

    inputs.trackingLostCounter = questNav.getTrackingLostCounter().orElse(-1);

    inputs.batteryPercent = questNav.getBatteryPercent().orElse(-1);

    inputs.latency = questNav.getLatency();

    inputs.currentlyTracking = questNav.isTracking();
    // Read new camera observations
    List<QuestNavObservation> questNavObservations = new LinkedList<>();

    for (var frame : questNav.getAllUnreadPoseFrames()) {
      processPoseFrames(frame, questNavObservations);
    }

    // Save pose observations to inputs object
    inputs.questNavObservations = new QuestNavObservation[questNavObservations.size()];
    for (int i = 0; i < questNavObservations.size(); i++) {
      inputs.questNavObservations[i] = questNavObservations.get(i);
    }
  }

  private void processPoseFrames(PoseFrame frame, List<QuestNavObservation> questNavObservations) {

    // Calculate robot pose
    Pose3d questPose = frame.questPose3d();
    Pose3d robotPose = questPose.plus(ROBOT_TO_QUEST.inverse());

    // Add observation
    questNavObservations.add(new QuestNavObservation(frame.dataTimestamp(), robotPose));
  }

  @Override
  public void setPose(Pose3d questNavTargetPose) {
    questNav.setPose(questNavTargetPose);
    Logger.recordOutput(
        "QuestNav/Log",
        String.format(
            "Pose reset requested to: (%.2f, %.2f, %.2f°)",
            questNavTargetPose.getX(),
            questNavTargetPose.getY(),
            questNavTargetPose.getRotation().getMeasureZ().in(Degrees)));
  }
}
