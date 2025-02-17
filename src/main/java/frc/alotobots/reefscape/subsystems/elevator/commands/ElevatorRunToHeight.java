/*
 * ALOTOBOTS - FRC Team 5152
 * https://github.com/5152Alotobots
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

/**
 * Command that moves the elevator to a specified target height.
 * Uses closed-loop control to accurately position the elevator and
 * completes when the target height is reached within tolerance.
 */
public class ElevatorRunToHeight extends Command {
  /** The elevator subsystem being controlled. */
  private final ElevatorSubsystem elevatorSubsystem;

  /** The target height for the elevator to reach. */
  private final Distance targetHeight;

  /**
   * Creates a new ElevatorRunToHeight command.
   *
   * @param elevatorSubsystem The elevator subsystem to control
   * @param targetHeight The desired height for the elevator to reach
   */
  public ElevatorRunToHeight(ElevatorSubsystem elevatorSubsystem, Distance targetHeight) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.targetHeight = targetHeight;
    addRequirements(elevatorSubsystem);
  }

  /**
   * Initializes the command by setting the elevator's target position.
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    elevatorSubsystem.runToTargetPosition(targetHeight);
  }

  /**
   * Executes the position control loop.
   * The PID control is handled internally by the subsystem.
   */
  @Override
  public void execute() {
    // Position control is handled by the subsystem's internal PID loop
  }

  /**
   * Called when the command ends or is interrupted.
   * Stops the elevator to ensure safe operation.
   *
   * @param interrupted true if the command was interrupted, false if it completed normally
   */
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  /**
   * Determines if the command has finished.
   * Returns true when the elevator has reached its target height within tolerance.
   *
   * @return true if the elevator is at the target height, false otherwise
   */
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.isAtTargetHeight();
  }
}