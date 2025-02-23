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

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;

/**
 * Command that holds the elevator at its current position. This command uses velocity control with
 * target speed of 0 and applies it to the elevator. ScheduleCommand to ensure that other commands
 * are able to interrupt it in a command group.
 */
public class ElevatorHoldHeight extends ScheduleCommand {
  /** The elevator subsystem this command controls. */
  private final ElevatorSubsystem elevatorSubsystem;

  /**
   * Creates a new ElevatorHoldHeight command. Should ONLY be used in autonomous. In teleop, the
   * default command replicates the same behavior.
   *
   * @param elevatorSubsystem The elevator subsystem this command will run on
   */
  public ElevatorHoldHeight(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;

    // This command requires the elevator subsystem
    addRequirements(elevatorSubsystem);
  }

  /**
   * Called when the command is initially scheduled. No initialization is needed for this command.
   */
  @Override
  public void initialize() {}

  /** Called repeatedly when this command is scheduled to run. */
  @Override
  public void execute() {
    elevatorSubsystem.runToTargetVelocity(MetersPerSecond.of(0));
  }

  /**
   * Called once when the command ends or is interrupted. Stops the elevator to ensure it doesn't
   * continue moving.
   *
   * @param interrupted Whether the command was interrupted (true) or completed normally (false)
   */
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  /**
   * Returns whether this command has finished. This command never finishes on its own and will run
   * until interrupted.
   *
   * @return false always, as this is a command that should not terminate
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
