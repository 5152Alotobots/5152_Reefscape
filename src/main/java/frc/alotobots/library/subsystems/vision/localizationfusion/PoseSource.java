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
package frc.alotobots.library.subsystems.vision.localizationfusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Standardized interface for pose estimation sources in the robot localization system.
 *
 * <p>This interface defines the contract that all pose sources must fulfill to integrate with the
 * localization fusion system. It supports various types of pose sources including:
 *
 * <p>- Meta Quest SLAM (primary source) - AprilTag vision (secondary source) - Wheel odometry
 * (emergency backup) - Other potential sources (e.g., LiDAR, external cameras)
 *
 * <p>Each source must provide: - Connection status monitoring - Current pose estimation -
 * Measurement uncertainty estimates - Source identification
 */
public interface PoseSource {

  /**
   * Checks if the pose source is currently connected and providing valid data.
   *
   * <p>This method should: - Verify hardware connectivity (if applicable) - Check data freshness -
   * Validate sensor readings - Monitor update frequency
   *
   * @return true if the source is connected and providing valid data
   */
  boolean isConnected();

  /**
   * Gets the most recent pose estimate from this source.
   *
   * <p>The returned pose should be: - In field-relative coordinates - Using the standard FRC
   * coordinate system: - Origin at field corner - +X towards opposite alliance wall - +Y towards
   * driver station - CCW positive rotation
   *
   * <p>If no valid pose is available, returns null.
   *
   * @return The current robot pose estimate, or null if unavailable
   */
  Pose2d getCurrentPose();

  /**
   * Gets the standard deviations of measurement uncertainty for this pose source.
   *
   * <p>Returns a 3x1 matrix containing standard deviations for: - X position (meters) - Y position
   * (meters) - Rotation (radians)
   *
   * <p>These values should: - Reflect real measurement uncertainty - Scale with distance/conditions
   * - Account for systematic errors - Consider environmental factors
   *
   * @return 3x1 matrix of standard deviations [x, y, theta]
   */
  Matrix<N3, N1> getStdDevs();

  /**
   * Gets a human-readable identifier for this pose source.
   *
   * <p>Used for: - Logging - Debugging - User interfaces - Status reporting
   *
   * @return String identifier for this pose source
   */
  String getSourceName();
}
