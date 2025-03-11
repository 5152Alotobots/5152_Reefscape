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
package frc.alotobots.library.subsystems.vision.localizationfusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public abstract class LocalizationSource {
  public Pose2d getPose() {
    // No extra processing needed right now, we just return the raw pose
    return getRawPose();
  }

  public Matrix<N3, N1> getStandardDeviations() {
    // No extra processing needed right now, we just return the raw standard deviations
    return getRawStdDevs();
  }

  public void update() {
    // Default implementation does nothing
  }

  protected abstract Pose2d getRawPose();

  protected abstract Matrix<N3, N1> getRawStdDevs();

  public abstract Time getTimestamp();

  public abstract LocalizationSourceState getState();

  protected abstract LocalizationType getType();
}
