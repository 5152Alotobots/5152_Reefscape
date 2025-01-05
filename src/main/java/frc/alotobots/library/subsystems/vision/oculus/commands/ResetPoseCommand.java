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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;

/**
 * Command that resets the Oculus system's pose estimation to a specified target pose. This is
 * useful for initializing or correcting the robot's position tracking.
 */
public class ResetPoseCommand extends Command {
  /** The Oculus subsystem instance to reset */
  private final OculusSubsystem oculus;

  /** The target pose to reset to */
  private final Pose2d targetPose;

  /**
   * Creates a new ResetPoseCommand.
   *
   * @param oculus The Oculus subsystem to reset
   * @param targetPose The desired pose to reset to
   */
  public ResetPoseCommand(OculusSubsystem oculus, Pose2d targetPose) {
    this.oculus = oculus;
    this.targetPose = targetPose;
    addRequirements(oculus);
  }

  @Override
  public void initialize() {
    oculus.resetToPose(targetPose);
  }

  @Override
  public boolean isFinished() {
    return !oculus.isPoseResetInProgress();
  }
}
