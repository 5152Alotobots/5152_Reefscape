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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;

/**
 * A command that disables the servos in the climber subsystem. This command executes instantly and
 * can run when the robot is disabled.
 */
public class ClimberDefaultPositions extends InstantCommand {
  /** The climber subsystem instance this command operates on */
  private final ClimberSubsystem climberSubsystem;

  /**
   * Creates a new ClimberDisableServos command.
   *
   * @param climberSubsystem The climber subsystem to control
   */
  public ClimberDefaultPositions(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  /**
   * Called when the command is initially scheduled. Disables the servos in the climber subsystem.
   */
  @Override
  public void initialize() {
    // climberSubsystem.enableServos();
    climberSubsystem.unlockCage();
    climberSubsystem.unlockElevator();
    // climberSubsystem.setPlungerToStow();
  }
}
