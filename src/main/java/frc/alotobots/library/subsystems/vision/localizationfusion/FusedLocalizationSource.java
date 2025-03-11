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

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import java.util.List;

public abstract class FusedLocalizationSource extends LocalizationSource {

  private final List<LocalizationSource> sources;
  private Pose2d lastFusedPose = new Pose2d();
  private Matrix<N3, N1> lastStdDevs = VecBuilder.fill(1000, 1000, 1000);
  private Time lastTimestamp = Seconds.of(-1);
  private LocalizationSourceState currentState = LocalizationSourceState.INITIALIZING;

  public FusedLocalizationSource(LocalizationSource... sources) {
    this.sources = List.of(sources);
  }

  @Override
  public Pose2d getPose() {
    return null;
  }

  @Override
  public Matrix<N3, N1> getStandardDeviations() {
    return null;
  }

  @Override
  public Time getTimestamp() {
    return null;
  }

  @Override
  public LocalizationSourceState getState() {
    return null;
  }

  @Override
  public LocalizationType getType() {
    return null;
  }

  @Override
  public void update() {}

  List<LocalizationSource> getSources() {
    return sources;
  }
}
