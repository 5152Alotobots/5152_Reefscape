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
package frc.alotobots.library.subsystems.vision.oculus.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;

public class RequestPositionResetViaAprilTags extends Command {

  private final double delayTime;
  private final OculusSubsystem oculusSubsystem;
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  public RequestPositionResetViaAprilTags(
      double delayTime,
      OculusSubsystem oculusSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem) {
    this.delayTime = delayTime;
    this.oculusSubsystem = oculusSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  Timer waitTimer = new Timer();

  @Override
  public void initialize() {
    waitTimer.restart();
  }

  @Override
  public void execute() {
    // If we have waited enough time, we can reset the oculus pose to that of the AprilTag pose.
    // This should only be called when the auto task is switched
    if (waitTimer.hasElapsed(delayTime) && DriverStation.isDisabled()) {
      oculusSubsystem.resetPose(swerveDriveSubsystem.getMultiTagPose());
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
