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
 * position the wrist. Can either end when reaching target or continuously hold position.
 */
public class WristRunToAngle extends Command {
  /** The wrist subsystem being controlled */
  private final WristSubsystem wristSubsystem;

  /** The target angle for the wrist to reach */
  private final Angle angle;

  private final boolean hasAlgae;

  /**
   * Creates a new WristRunToAngle command.
   *
   * @param wristSubsystem The wrist subsystem to control
   * @param angle The desired angle for the wrist to reach
   * @param hasAlgae Whether the robot has algae [TEMPORARY]
   */
  public WristRunToAngle(WristSubsystem wristSubsystem, Angle angle, boolean hasAlgae) {
    this.wristSubsystem = wristSubsystem;
    this.angle = angle;
    this.hasAlgae = hasAlgae;
    addRequirements(wristSubsystem);
  }

  /**
   * Creates a new WristRunToAngle command that ends after reaching target.
   *
   * @param wristSubsystem The wrist subsystem to control
   * @param angle The desired angle for the wrist to reach
   */
  public WristRunToAngle(WristSubsystem wristSubsystem, Angle angle) {
    this(wristSubsystem, angle, false);
  }

  @Override
  public void initialize() {
    wristSubsystem.runToTargetAngle(angle, hasAlgae);
  }

  @Override
  public void execute() {
    // Position control is handled by the subsystem's internal PID loop
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return wristSubsystem.isAtTargetAngle();
  }
}
