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
package frc.alotobots.library.subsystems.bling.commands;

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Animations.TIME_TO_CLIMB_BLUE_ANIMATION;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Animations.TIME_TO_CLIMB_RED_ANIMATION;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;

/**
 * Command that displays the Time To Climb animation for 2 seconds. This is triggered 5 seconds
 * before the end of the match.
 */
public class BlingTimeToClimb extends Command {
  /** The bling subsystem to control */
  private BlingSubsystem blingSubsystem;

  /**
   * Creates a new BlingTimeToClimb command.
   *
   * @param blingSubsystem The bling subsystem to use
   */
  public BlingTimeToClimb(BlingSubsystem blingSubsystem) {
    this.blingSubsystem = blingSubsystem;
    addRequirements(blingSubsystem);
    // Set a 2 second timeout for this command
  }

  @Override
  public void initialize() {
    // Select animation based on alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      blingSubsystem.setAnimation(TIME_TO_CLIMB_RED_ANIMATION);
    } else {
      blingSubsystem.setAnimation(TIME_TO_CLIMB_BLUE_ANIMATION);
    }
  }

  @Override
  public boolean isFinished() {
    // The timeout will handle ending the command
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    blingSubsystem.clear();
  }
}
