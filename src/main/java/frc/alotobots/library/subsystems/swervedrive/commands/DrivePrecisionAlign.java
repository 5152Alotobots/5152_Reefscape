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
package frc.alotobots.library.subsystems.swervedrive.commands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.function.Supplier;

/**
 * Handles precision alignment requests to a target pose using PathPlanner's holonomic drive
 * controller. Only attempts alignment when the robot is within a specified radius of the target.
 */
public class DrivePrecisionAlign {
  /** The swerve drive subsystem used for robot movement */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** The radius within which alignment will be attempted (meters) */
  private final double alignmentRadius;

  /** The position tolerance for considering alignment complete (meters) */
  private final double positionTolerance;

  /** The PathPlanner controller for holonomic movement */
  private final PPHolonomicDriveController controller;

  /** The trajectory state representing the target position and orientation */
  private final PathPlannerTrajectoryState targetTrajectoryState;

  /**
   * Creates a new PrecisionAlignRequest handler with custom alignment parameters.
   *
   * @param swerveDriveSubsystem The drive subsystem
   * @param alignmentRadius The radius within which alignment will be attempted (meters)
   * @param positionTolerance The position tolerance for considering alignment complete (meters)
   */
  public DrivePrecisionAlign(
      SwerveDriveSubsystem swerveDriveSubsystem, double alignmentRadius, double positionTolerance) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.alignmentRadius = alignmentRadius;
    this.positionTolerance = positionTolerance;
    this.controller = Constants.tunerConstants.getHolonomicDriveController();
    this.targetTrajectoryState = new PathPlannerTrajectoryState();
  }

  /**
   * Creates a new PrecisionAlignRequest handler with default parameters.
   *
   * @param swerveDriveSubsystem The drive subsystem
   */
  public DrivePrecisionAlign(SwerveDriveSubsystem swerveDriveSubsystem) {
    this(swerveDriveSubsystem, Constants.tunerConstants.getPrecisionAlignAllowRadius(), Constants.tunerConstants.getPrecisionAlignTolerance());
  }

  /** Called to initialize the request handler. */
  public void setup() {
    // Reset any state if needed for future implementations
  }

  /**
   * Applies the precision alignment request.
   *
   * @param targetPose Supplier for the target pose to align to
   */
  public void applyRequest(Supplier<Pose2d> targetPose) {
    Pose2d currentPose = swerveDriveSubsystem.getPose();
    Pose2d target = targetPose.get();
    double distance = currentPose.getTranslation().getDistance(target.getTranslation());

    // Only execute alignment if within radius
    if (distance <= alignmentRadius) {
      targetTrajectoryState.pose = target;
      ChassisSpeeds speeds =
          controller.calculateRobotRelativeSpeeds(currentPose, targetTrajectoryState);

      swerveDriveSubsystem.runVelocity(speeds);
    } else {
      swerveDriveSubsystem.stop();
    }
  }

  /**
   * Creates a command that executes the precision alignment.
   *
   * @param targetPose Supplier for the target pose to align to
   * @return A command that will execute the precision alignment
   */
  public Command getCommand(Supplier<Pose2d> targetPose) {
    return Commands.run(() -> applyRequest(targetPose), swerveDriveSubsystem)
        .beforeStarting(this::setup)
        .until(
            () -> {
              Pose2d currentPose = swerveDriveSubsystem.getPose();
              Pose2d target = targetPose.get();
              double distance = currentPose.getTranslation().getDistance(target.getTranslation());

              // End if outside radius or within tolerance
              return distance > alignmentRadius || distance < positionTolerance;
            });
  }

  /**
   * Creates a command that executes the precision alignment to a fixed pose.
   *
   * @param targetPose The fixed target pose to align to
   * @return A command that will execute the precision alignment
   */
  public Command getCommand(Pose2d targetPose) {
    return getCommand(() -> targetPose);
  }
}
