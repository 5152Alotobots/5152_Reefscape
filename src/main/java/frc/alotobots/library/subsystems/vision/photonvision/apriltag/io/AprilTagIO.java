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
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {

  @AutoLog
  public static class AprilTagIOInputs {
    public boolean connected = false;
    public SingleTagObservation[] singleTagObservations = new SingleTagObservation[0];
    public int[] singleTagIds = new int[0];
    public MultiTagObservation[] multiTagObservations = new MultiTagObservation[0];
    public int[] multiTagIds = new int[0];
    public Rotation2d[] singleTagHeadingBuffer = new Rotation2d[0];
  }

  /** Represents a robot pose sample used for pose estimation. (2+ tags) */
  public static record MultiTagObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  /** Represents a robot pose sample used for pose estimation. (1 tag) */
  public static record SingleTagObservation(
      double timestamp, Pose2d pose, double ambiguity, double tagDistance) {}

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
