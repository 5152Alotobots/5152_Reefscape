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
package frc.alotobots.library.subsystems.vision.localizationfusion.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusion;

/**
 * A command that requests a position reset using AprilTag detection. This command attempts to reset
 * the robot's position estimation using detected AprilTags through the LocalizationFusion
 * subsystem.
 */
public class RequestPositionResetViaTags extends Command {
  /** Flag indicating whether the reset has been successfully completed */
  private boolean reset = false;

  /** The LocalizationFusion subsystem instance */
  private final LocalizationFusion localizationFusion;

  /**
   * Creates a new RequestPositionResetViaTags command.
   *
   * @param localizationFusion The LocalizationFusion subsystem used for position reset
   */
  public RequestPositionResetViaTags(LocalizationFusion localizationFusion) {
    this.localizationFusion = localizationFusion;
    addRequirements(localizationFusion);
  }

  /** Initializes the command by resetting the completion flag. */
  @Override
  public void initialize() {
    reset = false;
  }

  /**
   * Executes the position reset request. Attempts to reset the Oculus pose estimation using
   * AprilTag detection.
   */
  @Override
  public void execute() {
    reset = localizationFusion.requestResetOculusPoseViaAprilTags();
  }

  /**
   * Determines if the command has finished.
   *
   * @return true if the position reset has been completed, false otherwise
   */
  @Override
  public boolean isFinished() {
    return reset;
  }

  /**
   * Specifies that this command can run when the robot is disabled.
   *
   * @return true, indicating the command can run while disabled
   */
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
