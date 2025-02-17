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

/**
 * Command that maintains the elevator's position at its current height. This command uses
 * closed-loop control to hold the elevator steady at whatever height it was at when the command was
 * initialized.
 */
public class ElevatorHoldHeight extends Command {
  /** The elevator subsystem being controlled. */
  private final ElevatorSubsystem elevatorSubsystem;

  /**
   * Creates a new ElevatorHoldHeight command.
   *
   * @param elevatorSubsystem The elevator subsystem to control
   */
  public ElevatorHoldHeight(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  /**
   * Initializes the command by setting the target position to the current height. Called when the
   * command is initially scheduled.
   */
  @Override
  public void initialize() {
    elevatorSubsystem.runToTargetPosition(elevatorSubsystem.getCurrentHeight());
  }

  /**
   * Executes the position holding control loop. The PID control is handled internally by the
   * subsystem.
   */
  @Override
  public void execute() {
    // Position control is handled by the subsystem's internal PID loop
  }

  /**
   * Called when the command ends or is interrupted. Stops the elevator to ensure safe operation.
   *
   * @param interrupted true if the command was interrupted, false if it completed normally
   */
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  /**
   * Determines if the command has finished. This command runs continuously until interrupted.
   *
   * @return false as this command runs until interrupted
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
