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
package frc.alotobots.library.subsystems.swervedrive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;

public class SeedFieldRelative extends InstantCommand {
  private SwerveDriveSubsystem swerveDriveSubsystem;

  public SeedFieldRelative(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    swerveDriveSubsystem.setPose(
        new Pose2d(swerveDriveSubsystem.getPose().getTranslation(), new Rotation2d()));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
