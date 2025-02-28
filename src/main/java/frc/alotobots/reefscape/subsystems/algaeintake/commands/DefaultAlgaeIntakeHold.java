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
package frc.alotobots.reefscape.subsystems.algaeintake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.algaeintake.AlgaeIntakeSubsystem;

public class DefaultAlgaeIntakeHold extends Command {

  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

  public DefaultAlgaeIntakeHold(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    addRequirements(algaeIntakeSubsystem);
  }

  @Override
  public void execute() {
    if (algaeIntakeSubsystem.isIntakeOccupied()) {
      algaeIntakeSubsystem.runAtPercentOutput(0.25);
    } else {
      algaeIntakeSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
