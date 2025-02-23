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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;

/**
 * Command that moves the wrist to a specified target angle. Uses closed-loop control to accurately
 * position the wrist and ends when the target angle is reached within tolerance.
 */
public class WristRunToAngle extends Command {
  /** The wrist subsystem being controlled */
  private final WristSubsystem wristSubsystem;

  /** The target angle for the wrist to reach */
  private final Angle angle;

  /**
   * Creates a new WristRunToAngle command.
   *
   * @param wristSubsystem The wrist subsystem to control
   * @param angle The desired angle for the wrist to reach
   */
  public WristRunToAngle(WristSubsystem wristSubsystem, Angle angle) {
    this.wristSubsystem = wristSubsystem;
    this.angle = angle;
    addRequirements(wristSubsystem);
  }

  /**
   * Initializes the command by setting the wrist's target position. Called when the command is
   * initially scheduled.
   */
  @Override
  public void initialize() {
    wristSubsystem.runToTargetAngle(angle);
  }

  /** Executes the position control loop. The PID control is handled internally by the subsystem. */
  @Override
  public void execute() {
    // Position control is handled by the subsystem's internal PID loop
  }

  /**
   * Called when the command ends or is interrupted. Stops the wrist to ensure safe operation.
   *
   * @param interrupted true if the command was interrupted, false if it completed normally
   */
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
  }

  /**
   * Determines if the command has finished. Returns true once angle is reached, then other
   * controller takes over
   *
   * @return true if at position
   */
  @Override
  public boolean isFinished() {
    return wristSubsystem.isAtTargetAngle();
  }
}
