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
 * Command that runs the intake in reverse (negative output) to eject game pieces out the front.
 * Automatically ends when the game piece is no longer detected by the intake sensor. The speed is
 * clamped to the maximum allowed eject percentage.
 */
public class CoralIntakeEjectManual extends Command {
  /** The coral intake subsystem being controlled */
  private final CoralIntakeSubsystem coralIntakeSubsystem;

  /** The input for controlling eject speed (converted to negative to push outward) */
  private final DoubleSupplier input;

  /**
   * Creates a new CoralIntakeEject command.
   *
   * @param coralIntakeSubsystem The intake subsystem to control
   * @param input Supplier for the eject speed (-MAX_OPEN_LOOP_EJECT_PERCENTAGE to 0.0). Input is
   *     made negative to push outward through the front of the intake.
   */
  public CoralIntakeEjectManual(CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier input) {
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.input = input;
    addRequirements(coralIntakeSubsystem);
  }

  /**
   * Runs the eject motors at the supplied speed in reverse, clamped to safe limits. Called
   * repeatedly while the command is scheduled.
   */
  @Override
  public void execute() {
    double adjustedOutput =
        MathUtil.clamp(-input.getAsDouble(), -MAX_OPEN_LOOP_EJECT_PERCENTAGE, 0);
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
    return false;
  }
}
