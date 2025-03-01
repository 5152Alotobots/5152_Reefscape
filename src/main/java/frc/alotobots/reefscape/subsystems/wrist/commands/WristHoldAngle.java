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

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;

/**
 * Command that holds the wrist at its current position. This command uses velocity control with
 * target speed of 0 and applies it to the wrist. ScheduleCommand to ensure that other commands are
 * able to interrupt it in a command group.
 */
public class WristHoldAngle extends ScheduleCommand {
  /** The elevator subsystem this command controls. */
  private final WristSubsystem wristSubsystem;

  /**
   * Creates a new WristHoldAngle command.
   *
   * @param wristSubsystem The wrist subsystem this command will run on
   */
  public WristHoldAngle(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;

    // This command requires the wrist subsystem
    addRequirements(wristSubsystem);
  }

  /**
   * Called when the command is initially scheduled. No initialization is needed for this command.
   */
  @Override
  public void initialize() {}

  /** Called repeatedly when this command is scheduled to run. */
  @Override
  public void execute() {
    wristSubsystem.runToTargetVelocity(DegreesPerSecond.of(0));
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
   * @return false always, as this is a command that should not terminate
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
