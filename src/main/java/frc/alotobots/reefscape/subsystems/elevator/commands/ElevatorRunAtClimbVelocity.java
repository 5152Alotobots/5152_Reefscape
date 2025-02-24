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

import static frc.alotobots.OI.AxisLimits.MAX_AXIS_LIMIT;
import static frc.alotobots.OI.AxisLimits.MIN_AXIS_LIMIT;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.MAX_SPEED;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command that runs the elevator at a specified velocity for climbing. This command takes a velocity
 * input (normalized between -1.0 and 1.0) and applies it to the elevator, scaled by the maximum
 * speed constant.
 */
public class ElevatorRunAtClimbVelocity extends Command {
  /** The elevator subsystem this command controls. */
  private final ElevatorSubsystem elevatorSubsystem;

  /**
   * Supplier for the input value that determines velocity direction and magnitude. Expected range
   * is between -1.0 and 1.0, where:
   *
   * <ul>
   *   <li>Positive values move the elevator upward
   *   <li>Negative values move the elevator downward
   *   <li>Zero stops the elevator
   * </ul>
   */
  private final DoubleSupplier input;

  /**
   * Creates a new ClimbingElevatorRunAtVelocity command.
   *
   * @param elevatorSubsystem The elevator subsystem this command will run on
   * @param input A supplier that provides the normalized velocity input (-1.0 to 1.0)
   */
  public ElevatorRunAtClimbVelocity(ElevatorSubsystem elevatorSubsystem, DoubleSupplier input) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.input = input;

    // This command requires the elevator subsystem
    addRequirements(elevatorSubsystem);
  }

  /**
   * Called when the command is initially scheduled. No initialization is needed for this command.
   */
  @Override
  public void initialize() {}

  /**
   * Called repeatedly when this command is scheduled to run. Calculates the target velocity by
   * multiplying the input value by the maximum speed, then commands the elevator subsystem to run
   * at that velocity.
   */
  @Override
  public void execute() {
    double adjustedInput = MathUtil.clamp(input.getAsDouble(), MIN_AXIS_LIMIT, MAX_AXIS_LIMIT);
    LinearVelocity velocity = MAX_SPEED.times(adjustedInput);
    elevatorSubsystem.runToClimbingVelocity(velocity);
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
   * @return false always, as this is a default command that should not terminate
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
