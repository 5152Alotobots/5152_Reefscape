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

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for handling input/output operations with the QuestNav hardware. */
public interface QuestNavIO {
  /** Data structure for QuestNav inputs that can be automatically logged. */
  @AutoLog
  public static class QuestNavIOInputs {

    /** Is the Quest currently connected */
    public boolean connected = false;

    /** Pose Frames */
    public QuestNavObservation[] questNavObservations = new QuestNavObservation[0];

    /** Total number of tracking lost events since the Quest has booted */
    public int trackingLostCounter = 0;

    /** Does the QuestNav have 6dof tracking? */
    public boolean currentlyTracking = false;

    /** Battery level percentage */
    public double batteryPercent = -1.0;

    /** Quest > Robot Latency in MS */
    public double latency = -1.0;
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record QuestNavObservation(double timestamp, Pose3d pose) {}

  /**
   * Updates the set of loggable inputs from the QuestNav.
   *
   * @param inputs The input object to update with current values
   */
  public default void updateInputs(QuestNavIOInputs inputs) {}

  /**
   * Resets the pose components for resetting the QuestNav position tracking. HARD RESET.
   *
   * @param questNavTargetPose The target pose of the quest to reset to. NOT THE TARGET ROBOT POSE
   */
  public default void setPose(Pose3d questNavTargetPose) {}
}
