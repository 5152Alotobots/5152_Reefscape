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

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Animations.NO_ALLIANCE_ANIMATION;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;

/**
 * Command that displays an animation while waiting for alliance selection. Automatically terminates
 * once an alliance is selected.
 */
public class NoAllianceWaiting extends Command {
  /** The bling subsystem to control */
  private BlingSubsystem blingSubsystem;

  /**
   * Creates a new NoAllianceWaiting command.
   *
   * @param blingSubsystem The bling subsystem to use
   */
  public NoAllianceWaiting(BlingSubsystem blingSubsystem) {
    this.blingSubsystem = blingSubsystem;
    addRequirements(blingSubsystem);
  }

  @Override
  public void initialize() {
    blingSubsystem.setAnimation(NO_ALLIANCE_ANIMATION);
  }

  @Override
  public boolean isFinished() {
    // Exit once we have an alliance
    return DriverStation.getAlliance().isPresent();
  }

  @Override
  public void end(boolean interrupted) {
    blingSubsystem.clear();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
