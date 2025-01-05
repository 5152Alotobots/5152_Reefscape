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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.util.DriveCalculator;

public class DefaultDrive {
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  public DefaultDrive(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  public void applyRequest() {
    // Just run the speeds :)
    swerveDriveSubsystem.runVelocity(DriveCalculator.getChassisSpeeds(swerveDriveSubsystem));
  }

  public Command getCommand() {
    return Commands.run(this::applyRequest, swerveDriveSubsystem);
  }
}
