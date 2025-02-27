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

/**
 * Command that moves the elevator to a specified target height. Uses closed-loop control to
 * accurately position the elevator. Can either end when reaching target or continuously hold
 * position.
 */
public class ElevatorRunToHeight extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final Distance targetHeight;
  private final boolean holdPosition;

  /**
   * Creates a new ElevatorRunToHeight command.
   *
   * @param elevatorSubsystem The elevator subsystem to control
   * @param targetHeight The desired height for the elevator to reach
   * @param holdPosition If true, command will continue running to maintain position
   */
  public ElevatorRunToHeight(
      ElevatorSubsystem elevatorSubsystem, Distance targetHeight, boolean holdPosition) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.targetHeight = targetHeight;
    this.holdPosition = holdPosition;
    addRequirements(elevatorSubsystem);
  }

  // Constructor overload for backward compatibility
  public ElevatorRunToHeight(ElevatorSubsystem elevatorSubsystem, Distance targetHeight) {
    this(elevatorSubsystem, targetHeight, false);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.runToTargetPosition(targetHeight);
  }

  @Override
  public void execute() {
    // Position control handled by subsystem
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return !holdPosition && elevatorSubsystem.isAtTargetHeight();
  }
}
