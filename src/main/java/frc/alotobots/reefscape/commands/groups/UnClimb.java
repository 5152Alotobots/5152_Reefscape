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
package frc.alotobots.reefscape.commands.groups;

import edu.wpi.first.wpilibj2.command.*;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;

/**
 * Default command that runs the elevator at a specified velocity. This command takes a velocity
 * input (normalized between -1.0 and 1.0) and applies it to the elevator, scaled by the maximum
 * speed constant.
 */
public class UnClimb extends SequentialCommandGroup {

  public UnClimb(ClimberSubsystem climberSubsystem) {

    addCommands(
        new InstantCommand(climberSubsystem::enableServos),
        new InstantCommand(climberSubsystem::setPlungerToReceive),
        new InstantCommand(climberSubsystem::unlockCage),
        new WaitCommand(1),
        new InstantCommand(climberSubsystem::disableServos)
        );
    addRequirements(climberSubsystem);
  }
}
