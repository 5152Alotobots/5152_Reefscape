/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
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

public class ElevatorHoldHeight extends Command {
  private final ElevatorSubsystem elevatorSubsystem;

  /**
   * Creates a new ElevatorHoldHeight command.
   * This command will maintain the elevator's height at wherever it currently is
   * when the command is initialized.
   *
   * @param elevatorSubsystem The elevator subsystem to use
   */
  public ElevatorHoldHeight(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    // Get current height and set it as target
    elevatorSubsystem.runToTargetPosition(elevatorSubsystem.getCurrentHeight());
  }

  @Override
  public void execute() {
    // The runToTargetPosition method handles the PID control internally
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Command never finishes on its own - will keep holding height
    return false;
  }
}