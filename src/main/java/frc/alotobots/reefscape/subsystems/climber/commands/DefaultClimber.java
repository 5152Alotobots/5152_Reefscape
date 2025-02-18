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
package frc.alotobots.reefscape.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;

/**
 * Default command that runs the elevator at a specified velocity. This command takes a velocity
 * input (normalized between -1.0 and 1.0) and applies it to the elevator, scaled by the maximum
 * speed constant.
 */
public class DefaultClimber extends Command {
  /** The elevator subsystem this command controls. */
  private final ClimberSubsystem climberSubsystem;

  public DefaultClimber(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  /**
   * Called when the command is initially scheduled. No initialization is needed for this command.
   */
  @Override
  public void initialize() {

    climberSubsystem.climb();
  }

  /**
   * Called repeatedly when this command is scheduled to run. Calculates the target velocity by
   * multiplying the input value by the maximum speed, then commands the elevator subsystem to run
   * at that velocity.
   */
  @Override
  public void execute() {}

  /**
   * Called once when the command ends or is interrupted. Stops the elevator to ensure it doesn't
   * continue moving.
   *
   * @param interrupted Whether the command was interrupted (true) or completed normally (false)
   */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns whether this command has finished. This command never finishes on its own and will run
   * until interrupted.
   *
   * @return false always, as this is a default command that should not terminate
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
