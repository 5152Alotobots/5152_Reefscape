/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.oculus.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;

/**
 * Command that zeros the heading (rotation) of the Oculus system. This is useful for aligning the
 * robot's coordinate system with the field.
 */
public class ZeroHeadingCommand extends Command {
  /** The Oculus subsystem instance to zero */
  private final OculusSubsystem oculus;

  /**
   * Creates a new ZeroHeadingCommand.
   *
   * @param oculus The Oculus subsystem to zero
   */
  public ZeroHeadingCommand(OculusSubsystem oculus) {
    this.oculus = oculus;
    addRequirements(oculus);
  }

  @Override
  public void initialize() {
    oculus.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return !oculus.isHeadingResetInProgress();
  }
}
