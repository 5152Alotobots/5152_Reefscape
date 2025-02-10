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
package frc.alotobots.reefscape.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultElevatorOpenLoop extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final DoubleSupplier percent;

  public DefaultElevatorOpenLoop(ElevatorSubsystem elevatorSubsystem, DoubleSupplier percent) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.percent = percent;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.runAtPercentOutput(percent.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
