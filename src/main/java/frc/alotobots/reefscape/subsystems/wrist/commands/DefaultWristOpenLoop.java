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

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Default command for controlling the wrist in open-loop mode. This command runs when no other
 * command is using the wrist subsystem.
 */
public class DefaultWristOpenLoop extends Command {
  /** The wrist subsystem being controlled */
  private final WristSubsystem wristSubsystem;

  /** Supplier for the percent output value */
  private final DoubleSupplier input;

  /**
   * Creates a new DefaultWristOpenLoop command.
   *
   * @param wristSubsystem The wrist subsystem to control
   * @param input Supplier for the percent output (-1.0 to 1.0)
   */
  public DefaultWristOpenLoop(WristSubsystem wristSubsystem, DoubleSupplier input) {
    this.wristSubsystem = wristSubsystem;
    this.input = input;

    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wristSubsystem.runAtPercentOutput(input.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stop();
  }
}
