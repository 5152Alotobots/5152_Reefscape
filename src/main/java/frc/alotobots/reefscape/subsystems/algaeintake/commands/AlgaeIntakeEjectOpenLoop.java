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
package frc.alotobots.reefscape.subsystems.algaeintake.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.algaeintake.AlgaeIntakeSubsystem;

import java.util.function.DoubleSupplier;

import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeConstants.Limits.MAX_OPEN_LOOP_OUTTAKE_PERCENTAGE;

/**
 * Command that runs the algae intake in reverse (negative output) to eject game pieces.
 * Automatically ends when the game piece is no longer detected by the intake sensor.
 * The speed is clamped to the maximum allowed eject percentage.
 */
public class AlgaeIntakeEjectOpenLoop extends Command {
  /** The algae intake subsystem being controlled */
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

  /** The input for controlling eject speed */
  private final DoubleSupplier input;

  /**
   * Creates a new AlgaeIntakeEjectOpenLoop command.
   *
   * @param algaeIntakeSubsystem The intake subsystem to control
   * @param input Supplier for the eject speed (0.0 to MAX_OPEN_LOOP_OUTTAKE_PERCENTAGE).
   *     Input is made negative to push outward.
   */
  public AlgaeIntakeEjectOpenLoop(AlgaeIntakeSubsystem algaeIntakeSubsystem, DoubleSupplier input) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    this.input = input;
    addRequirements(algaeIntakeSubsystem);
  }

  /**
   * Runs the intake motors at the supplied speed in reverse, clamped to safe limits.
   * Called repeatedly while the command is scheduled.
   */
  @Override
  public void execute() {
    // Take the input speed, clamp it, and make it negative for outtake motion
    double adjustedOutput = -MathUtil.clamp(input.getAsDouble(), 0, MAX_OPEN_LOOP_OUTTAKE_PERCENTAGE);
    algaeIntakeSubsystem.runAtPercentOutput(adjustedOutput);
  }

  /**
   * Called when the command ends or is interrupted. Stops the intake motors.
   *
   * @param interrupted true if the command was interrupted, false if it completed normally
   */
  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.stop();
  }

  /**
   * Determines if the command has finished. Returns true once the game piece is no longer detected.
   *
   * @return true if no game piece is detected in the intake
   */
  @Override
  public boolean isFinished() {
    return !algaeIntakeSubsystem.isIntakeOccupied();
  }
}