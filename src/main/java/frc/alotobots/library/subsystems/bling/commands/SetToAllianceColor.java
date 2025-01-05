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
package frc.alotobots.library.subsystems.bling.commands;

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Colors.BLUE_ALLIANCE_COLOR;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Colors.RED_ALLIANCE_COLOR;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;

/**
 * Command that sets the LED color based on the current alliance color. Changes the LEDs to red or
 * blue depending on the alliance selection.
 */
public class SetToAllianceColor extends Command {
  /** The bling subsystem to control */
  private BlingSubsystem blingSubsystem;

  /**
   * Creates a new SetToAllianceColor command.
   *
   * @param blingSubsystem The bling subsystem to use
   */
  public SetToAllianceColor(BlingSubsystem blingSubsystem) {
    this.blingSubsystem = blingSubsystem;
    addRequirements(blingSubsystem);
  }

  @Override
  public void initialize() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
        blingSubsystem.setSolidColor(RED_ALLIANCE_COLOR);
      } else {
        blingSubsystem.setSolidColor(BLUE_ALLIANCE_COLOR);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    blingSubsystem.clear();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
