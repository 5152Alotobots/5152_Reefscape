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
package frc.alotobots.library.subsystems.vision.oculus.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.alotobots.library.subsystems.vision.localizationfusion.PoseSource;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants;

/**
 * Adapts the Meta Quest SLAM system to the standardized PoseSource interface.
 *
 * <p>This class wraps the OculusSubsystem to provide high-frequency pose estimation through the
 * standardized PoseSource interface. The Quest serves as: - Primary pose estimation source -
 * High-frequency (120Hz) updates - Global SLAM-based localization - Drift-free position tracking
 *
 * <p>Features: - Sub-millimeter tracking precision - Building-scale SLAM mapping - Robust to
 * reflective surfaces - Fast relocalization from saved maps - High-speed movement handling
 */
public class OculusPoseSource implements PoseSource {
  /** The underlying Oculus subsystem */
  public final OculusSubsystem subsystem;

  /**
   * Creates a new OculusPoseSource.
   *
   * @param subsystem The Oculus subsystem to wrap
   */
  public OculusPoseSource(OculusSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  /**
   * {@inheritDoc}
   *
   * <p>For Quest SLAM, connection status depends on: - Quest hardware connection - SLAM tracking
   * quality - Update frequency - Environment mapping status
   */
  @Override
  public boolean isConnected() {
    return subsystem.isConnected();
  }

  /**
   * {@inheritDoc}
   *
   * <p>Quest poses are: - Globally consistent via SLAM - Updated at 120Hz - Drift-compensated -
   * Environment-referenced - Transform-corrected to field frame
   */
  @Override
  public Pose2d getCurrentPose() {
    return subsystem.getCurrentPose();
  }

  /**
   * {@inheritDoc}
   *
   * <p>Quest measurement uncertainty: - Sub-millimeter position accuracy - Factory-calibrated IMU -
   * Environment-dependent SLAM quality - Constant across operating range
   */
  @Override
  public Matrix<N3, N1> getStdDevs() {
    return OculusConstants.OCULUS_STD_DEVS;
  }

  /** {@inheritDoc} */
  @Override
  public String getSourceName() {
    return "Oculus Quest";
  }
}
