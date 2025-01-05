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
package frc.alotobots.library.subsystems.vision.localizationfusion.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusion;

public class RequestPositionResetViaTags extends Command {
  boolean reset = false;
  LocalizationFusion localizationFusion;

  public RequestPositionResetViaTags(LocalizationFusion localizationFusion) {
    this.localizationFusion = localizationFusion;
    addRequirements(localizationFusion);
  }

  @Override
  public void initialize() {
    reset = false;
  }

  @Override
  public void execute() {
    reset = localizationFusion.requestResetOculusPoseViaAprilTags();
  }

  @Override
  public boolean isFinished() {
    return reset;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
