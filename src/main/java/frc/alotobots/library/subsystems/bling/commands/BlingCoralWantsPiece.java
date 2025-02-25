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

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.Animations.CORAL_WANTS_PIECE_ANIMATION;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;

/**
 * Command that displays the Coral Wants Piece animation until a piece is detected. Should be ran in
 * a race group.
 */
public class BlingCoralWantsPiece extends Command {
  /** The bling subsystem to control */
  private BlingSubsystem blingSubsystem;

  /**
   * Creates a new BlingCoralWantsPiece command.
   *
   * @param blingSubsystem The bling subsystem to use
   */
  public BlingCoralWantsPiece(BlingSubsystem blingSubsystem) {
    this.blingSubsystem = blingSubsystem;
    addRequirements(blingSubsystem);
  }

  @Override
  public void initialize() {
    blingSubsystem.setAnimation(CORAL_WANTS_PIECE_ANIMATION);
  }

  @Override
  public boolean isFinished() {
    // End when the intake has a piece
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    blingSubsystem.clear();
  }
}
