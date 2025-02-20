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

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Limits.MAX_OPEN_LOOP_EJECT_PERCENTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command that runs the intake to eject game pieces by pulling them through in the intake direction
 * (positive output). This runs the motors in the same direction as intake to push pieces out
 * through the intake side of the robot. Automatically ends when the game piece is no longer
 * detected by the intake sensor.
 */
public class CoralIntakeEjectThrough extends Command {
  /** The coral intake subsystem being controlled */
  private final CoralIntakeSubsystem coralIntakeSubsystem;

  /**
   * The input for controlling through-eject speed (positive values pull through in intake
   * direction)
   */
  private final DoubleSupplier input;

  /**
   * Creates a new CoralIntakeEjectThrough command.
   *
   * @param coralIntakeSubsystem The intake subsystem to control
   * @param input Supplier for the eject speed (0.0 to MAX_OPEN_LOOP_EJECT_PERCENTAGE). Positive
   *     values pull through in the intake direction.
   */
  public CoralIntakeEjectThrough(CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier input) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.input = input;
    addRequirements(coralIntakeSubsystem);
  }

  /**
   * Runs the motors at the supplied speed in the intake direction, clamped to safe limits. Called
   * repeatedly while the command is scheduled.
   */
  @Override
  public void execute() {
    double adjustedOutput = MathUtil.clamp(input.getAsDouble(), 0, MAX_OPEN_LOOP_EJECT_PERCENTAGE);
    coralIntakeSubsystem.runAtPercentOutput(adjustedOutput);
  }

  /**
   * Called when the command ends or is interrupted. Stops the intake motors.
   *
   * @param interrupted true if the command was interrupted, false if it completed normally
   */
  @Override
  public void end(boolean interrupted) {
    coralIntakeSubsystem.stop();
  }

  /**
   * Determines if the command has finished. Returns true once the game piece is no longer detected.
   *
   * @return true if no game piece is detected in the intake
   */
  @Override
  public boolean isFinished() {
    return !coralIntakeSubsystem.isIntakeOccupied();
  }
}
