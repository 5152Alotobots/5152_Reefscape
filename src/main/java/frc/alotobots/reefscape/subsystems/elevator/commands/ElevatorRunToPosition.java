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

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;

public class ElevatorRunToPosition extends Command {
  private final ElevatorSubsystem elevator;
  private final Distance targetHeight;

  /**
   * Creates a new ElevatorToHeightCommand.
   *
   * @param elevator The elevator subsystem to use
   * @param targetHeight The target height in meters
   */
  public ElevatorRunToPosition(ElevatorSubsystem elevator, Distance targetHeight) {
    this.elevator = elevator;
    this.targetHeight = targetHeight;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.runToTargetPosition(targetHeight);
  }

  @Override
  public void execute() {
    // The runToTargetPosition method handles the PID control internally
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    // TODO: Add position tolerance check based on your requirements
    // Example: return Math.abs(elevator.getCurrentHeight() - targetHeight) < TOLERANCE;
    return false;
  }
}
