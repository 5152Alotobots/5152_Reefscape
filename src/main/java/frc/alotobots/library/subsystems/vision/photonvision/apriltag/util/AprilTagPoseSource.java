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
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.alotobots.library.subsystems.vision.localizationfusion.PoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;

/**
 * Adapts the AprilTag vision system to the standardized PoseSource interface.
 *
 * <p>This class wraps the AprilTagSubsystem to provide pose estimation through the standardized
 * PoseSource interface. AprilTags serve as: - Secondary pose validation source - Initial pose
 * calibration reference - Backup pose estimation when Quest is unavailable
 *
 * <p>Pose accuracy depends on: - Number of visible AprilTags - Distance to detected tags - Camera
 * calibration quality - Tag pose ambiguity factors
 */
public class AprilTagPoseSource implements PoseSource {
  /** The underlying AprilTag subsystem */
  private final AprilTagSubsystem subsystem;

  /**
   * Creates a new AprilTagPoseSource.
   *
   * @param subsystem The AprilTag subsystem to wrap
   */
  public AprilTagPoseSource(AprilTagSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  /**
   * {@inheritDoc}
   *
   * <p>For AprilTags, connection status depends on: - Camera connection status - Valid pose
   * availability - Recent detection history
   */
  @Override
  public boolean isConnected() {
    return subsystem.isConnected();
  }

  /**
   * {@inheritDoc}
   *
   * <p>AprilTag poses are: - Field-relative - Validated for ambiguity - Filtered for physical
   * feasibility - Updated at camera frame rate
   */
  @Override
  public Pose2d getCurrentPose() {
    return subsystem.getCurrentPose();
  }

  /**
   * {@inheritDoc}
   *
   * <p>Measurement uncertainty scales with: - Tag distance - Number of visible tags - Viewing angle
   * - Tag pose ambiguity
   */
  @Override
  public Matrix<N3, N1> getStdDevs() {
    return subsystem.getStdDevs();
  }

  /** {@inheritDoc} */
  @Override
  public String getSourceName() {
    return "AprilTag";
  }
}
