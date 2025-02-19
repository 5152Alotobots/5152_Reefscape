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
package frc.alotobots.reefscape.subsystems.coralIntake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DefaultCoralIntakeOpenLoopWOLimits extends Command {
  private final CoralIntakeSubsystem coralIntakeSubsystem;
  private final DoubleSupplier input;

  public DefaultCoralIntakeOpenLoopWOLimits(
      CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier input) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.input = input;

    addRequirements(coralIntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Logger.recordOutput("Wrist/PercentOut", input.getAsDouble());
    coralIntakeSubsystem.runAtPercentOutput(input.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    coralIntakeSubsystem.stop();
  }
}
