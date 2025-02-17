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

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

/**
 * A default command for manually controlling the elevator using open-loop control.
 * This command allows direct control of the elevator's motor output using a percent value,
 * typically bound to a joystick or other controller input.
 */
public class DefaultElevatorOpenLoop extends Command {
  /** The elevator subsystem being controlled. */
  private final ElevatorSubsystem elevatorSubsystem;

  /** Supplier for the percent output value, typically from a controller input. */
  private final DoubleSupplier percent;

  /**
   * Creates a new DefaultElevatorOpenLoop command.
   *
   * @param elevatorSubsystem The elevator subsystem to be controlled
   * @param percent A supplier for the percent output value (-1.0 to 1.0)
   */
  public DefaultElevatorOpenLoop(ElevatorSubsystem elevatorSubsystem, DoubleSupplier percent) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.percent = percent;
    addRequirements(elevatorSubsystem);
  }

  /**
   * Called when the command is initially scheduled.
   * No initialization is required for this command.
   */
  @Override
  public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Updates the elevator's output based on the current percent value from the supplier.
   */
  @Override
  public void execute() {
    elevatorSubsystem.runAtPercentOutput(percent.getAsDouble());
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops the elevator to prevent uncontrolled motion.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  /**
   * Returns whether this command has finished.
   * This command never finishes on its own as it's designed to run continuously.
   *
   * @return false since this is a continuous command
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}