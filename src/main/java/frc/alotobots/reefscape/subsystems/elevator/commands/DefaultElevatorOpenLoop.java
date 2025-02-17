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
import java.util.function.DoubleSupplier;

/**
 * Default command for manual control of the elevator using percent output. This command is
 * typically used as the default command for the elevator subsystem, allowing direct operator
 * control through a joystick or other input device.
 */
public class DefaultElevatorOpenLoop extends Command {
  /** The elevator subsystem being controlled. */
  private final ElevatorSubsystem elevatorSubsystem;

  /** Provides the control input value, typically from a joystick or controller. */
  private final DoubleSupplier percent;

  /**
   * Creates a new DefaultElevatorOpenLoop command for manual elevator control.
   *
   * @param elevatorSubsystem The elevator subsystem to control
   * @param percent Supplier for the control input (-1.0 to 1.0), typically from a joystick
   */
  public DefaultElevatorOpenLoop(ElevatorSubsystem elevatorSubsystem, DoubleSupplier percent) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.percent = percent;
    addRequirements(elevatorSubsystem);
  }

  /**
   * Called when the command is initially scheduled. No initialization is required for this command.
   */
  @Override
  public void initialize() {}

  /**
   * Executes the manual control loop. Called every scheduler run while the command is scheduled.
   */
  @Override
  public void execute() {
    elevatorSubsystem.runAtPercentOutput(percent.getAsDouble());
  }

  /**
   * Called when the command ends or is interrupted. Stops the elevator to prevent uncontrolled
   * motion.
   *
   * @param interrupted true if the command was interrupted, false if it completed normally
   */
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stop();
  }

  /**
   * Determines if the command has finished. This command runs until interrupted, so it never
   * finishes on its own.
   *
   * @return false as this command runs continuously
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
