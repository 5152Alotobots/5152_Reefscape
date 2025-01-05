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
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagIO {

  @AutoLog
  public static class AprilTagIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  public default void updateInputs(AprilTagIOInputs inputs) {}
}
