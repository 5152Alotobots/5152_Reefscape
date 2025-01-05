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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.util.DriveCalculator;
import java.util.function.Supplier;

public class DriveFacingAngle {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final ProfiledPIDController angleController;

  /**
   * Creates a new DriveFacingAngle request.
   *
   * @param swerveDriveSubsystem The drive subsystem
   */
  public DriveFacingAngle(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;

    // Use current angle controller
    this.angleController = Constants.tunerConstants.getDriveFacingAnglePIDController();
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setup() {
    angleController.reset(swerveDriveSubsystem.getRotation().getRadians());
  }

  public void applyRequest(Supplier<Rotation2d> targetRotation) {
    // Calculate angular speed
    double omega =
        angleController.calculate(
            swerveDriveSubsystem.getRotation().getRadians(), targetRotation.get().getRadians());

    // Get the chassis speeds while overriding rotational
    ChassisSpeeds speeds =
        DriveCalculator.getChassisSpeedsWithRotationOverride(swerveDriveSubsystem, omega);
    swerveDriveSubsystem.runVelocity(speeds);
  }

  public Command getCommand(Supplier<Rotation2d> targetRotation) {
    return Commands.startRun(this::setup, () -> applyRequest(targetRotation), swerveDriveSubsystem);
  }
}
