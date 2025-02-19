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
package frc.alotobots.reefscape.subsystems.wrist.commands;

import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.MAX_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Default command that runs the wrist at a specified velocity. This command takes a velocity input
 * (normalized between -1.0 and 1.0) and applies it to the wrist, scaled by the maximum speed
 * constant.
 */
public class DefaultWristRunAtVelocity extends Command {
  /** The elevator subsystem this command controls. */
  private final WristSubsystem wristSubsystem;

  /**
   * Supplier for the input value that determines velocity direction and magnitude. Expected range
   * is between -1.0 and 1.0, where:
   *
   * <ul>
   *   <li>Positive values move the wrist downwards
   *   <li>Negative values move the wrist upwards
   *   <li>Zero stops the elevator
   * </ul>
   */
  private final DoubleSupplier input;

  /**
   * Creates a new DefaultWristRunAtVelocity command.
   *
   * @param wristSubsystem The wrist subsystem this command will run on
   * @param input A supplier that provides the normalized velocity input (-1.0 to 1.0)
   */
  public DefaultWristRunAtVelocity(WristSubsystem wristSubsystem, DoubleSupplier input) {
    this.wristSubsystem = wristSubsystem;
    this.input = input;

    // This command requires the wrist subsystem
    addRequirements(wristSubsystem);
  }

  /**
   * Called when the command is initially scheduled. No initialization is needed for this command.
   */
  @Override
  public void initialize() {}

  /**
   * Called repeatedly when this command is scheduled to run. Calculates the target velocity by
   * multiplying the input value by the maximum speed, then commands the wrist subsystem to run at
   * that velocity.
   */
  @Override
  public void execute() {
    double adjustedInput = MathUtil.clamp(input.getAsDouble(), -1, 1);
    AngularVelocity velocity = MAX_SPEED.times(adjustedInput);
    wristSubsystem.runToTargetVelocity(velocity);
  }

  /**
   * Called once when the command ends or is interrupted. Stops the wrist to ensure it doesn't
   * continue moving.
   *
   * @param interrupted Whether the command was interrupted (true) or completed normally (false)
   */
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
  }

  /**
   * Returns whether this command has finished. This command never finishes on its own and will run
   * until interrupted.
   *
   * @return false always, as this is a default command that should not terminate
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
