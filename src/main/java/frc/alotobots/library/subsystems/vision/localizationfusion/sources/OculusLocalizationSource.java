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
package frc.alotobots.library.subsystems.vision.localizationfusion.sources;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationSource;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationSourceState;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationType;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants;

public class OculusLocalizationSource extends LocalizationSource {

  private final OculusSubsystem oculusSubsystem;

  public OculusLocalizationSource(OculusSubsystem oculusSubsystem) {
    this.oculusSubsystem = oculusSubsystem;
  }

  @Override
  protected Pose2d getRawPose() {
    return oculusSubsystem.getPose();
  }

  @Override
  protected Matrix<N3, N1> getRawStdDevs() {
    return OculusConstants.OCULUS_STD_DEVS;
  }

  @Override
  public Time getTimestamp() {
    return oculusSubsystem.getTimestamp();
  }

  @Override
  public LocalizationSourceState getState() {
    return LocalizationSourceState.DISABLED;
  }

  @Override
  protected LocalizationType getType() {
    return null;
  }
}
