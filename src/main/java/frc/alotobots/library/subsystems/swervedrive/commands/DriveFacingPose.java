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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.util.DriveCalculator;
import java.util.function.Supplier;

public class DriveFacingPose {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final ProfiledPIDController angleController;

  /**
   * Creates a new DriveFacingPose command.
   *
   * @param swerveDriveSubsystem The drive subsystem
   */
  public DriveFacingPose(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;

    // Use current angle controller
    this.angleController = Constants.tunerConstants.getDriveFacingAnglePIDController();
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setup() {
    angleController.reset(swerveDriveSubsystem.getRotation().getRadians());
  }

  public void applyRequest(Supplier<Pose2d> targetPose) {
    // Calculate angle to target pose
    Pose2d currentPose = swerveDriveSubsystem.getPose();
    Pose2d target = targetPose.get();

    // Calculate the angle between current position and target position
    double dx = target.getX() - currentPose.getX();
    double dy = target.getY() - currentPose.getY();
    Rotation2d angleToTarget = new Rotation2d(Math.atan2(dy, dx));

    // Calculate angular speed
    double omega =
        angleController.calculate(
            swerveDriveSubsystem.getRotation().getRadians(), angleToTarget.getRadians());

    // Get the chassis speeds while overriding rotational
    ChassisSpeeds speeds =
        DriveCalculator.getChassisSpeedsWithRotationOverride(swerveDriveSubsystem, omega);
    swerveDriveSubsystem.runVelocity(speeds);
  }

  public Command getCommand(Supplier<Pose2d> targetPose) {
    return Commands.startRun(this::setup, () -> applyRequest(targetPose), swerveDriveSubsystem);
  }
}
