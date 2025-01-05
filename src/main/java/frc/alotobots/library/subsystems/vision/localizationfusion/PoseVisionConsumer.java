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

/** Functional interface for consuming pose updates from the fusion system. */
@FunctionalInterface
public interface PoseVisionConsumer {
  /**
   * Accepts a new pose update from the fusion system.
   *
   * @param pose The current robot pose in field coordinates
   * @param timestampSeconds The timestamp when this pose was measured
   * @param stdDevs Standard deviations for x, y, and rotation measurements
   */
  void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
}
