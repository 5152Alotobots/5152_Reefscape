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
 * A sequential command group that handles the unclimbing sequence. This command executes a series
 * of steps to safely disengage the climbing mechanism: 1. Enables the servos 2. Sets the plunger to
 * receive position 3. Unlocks the cage 4. Waits for 1 second 5. Disables the servos
 */
public class UnClimb extends SequentialCommandGroup {

  /**
   * Creates a new UnClimb command.
   *
   * @param climberSubsystem The climber subsystem to control
   */
  public UnClimb(ClimberSubsystem climberSubsystem) {
    addCommands(
        new InstantCommand(climberSubsystem::enableServos),
        new InstantCommand(climberSubsystem::setPlungerToReceive),
        new InstantCommand(climberSubsystem::unlockCage),
        new WaitCommand(1),
        new InstantCommand(climberSubsystem::disableServos));
    addRequirements(climberSubsystem);
  }
}
