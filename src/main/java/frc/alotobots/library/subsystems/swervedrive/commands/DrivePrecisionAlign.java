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
import lombok.Getter;
import lombok.Setter;

/**
 * Handles precision alignment requests to a target pose using PathPlanner's holonomic drive
 * controller.
 */
public class DrivePrecisionAlign {
  /** The swerve drive subsystem used for robot movement */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** The PathPlanner controller for holonomic movement */
  private final PPHolonomicDriveController controller;

  /** The trajectory state representing the target position and orientation */
  private final PathPlannerTrajectoryState targetTrajectoryState;

  /** The position tolerance for considering alignment complete (meters) */
  private final double positionTolerance;

  /**
   * The current target pose being tracked -- SETTER -- Sets the current target pose.
   *
   * <p>-- GETTER -- Gets the current target pose being tracked.
   *
   * @param targetPose The target pose to set
   * @return The current target pose
   */
  @Getter @Setter private Pose2d currentTargetPose;

  /**
   * Creates a new PrecisionAlignRequest handler.
   *
   * @param swerveDriveSubsystem The drive subsystem
   */
  /**
   * Creates a new PrecisionAlignRequest handler.
   *
   * @param swerveDriveSubsystem The drive subsystem
   * @param positionTolerance The position tolerance for considering alignment complete (meters)
   */
  public DrivePrecisionAlign(SwerveDriveSubsystem swerveDriveSubsystem, double positionTolerance) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.positionTolerance = positionTolerance;
    this.controller = Constants.tunerConstants.getHolonomicDriveController();
    this.targetTrajectoryState = new PathPlannerTrajectoryState();
    this.currentTargetPose = new Pose2d(); // Initialize with default pose
  }

  /**
   * Creates a new PrecisionAlignRequest handler with default tolerance.
   *
   * @param swerveDriveSubsystem The drive subsystem
   */
  public DrivePrecisionAlign(SwerveDriveSubsystem swerveDriveSubsystem) {
    this(swerveDriveSubsystem, Constants.tunerConstants.getPrecisionAlignTolerance());
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
    this.currentTargetPose = targetPose.get();

    targetTrajectoryState.pose = currentTargetPose;
    ChassisSpeeds speeds =
        controller.calculateRobotRelativeSpeeds(currentPose, targetTrajectoryState);

    swerveDriveSubsystem.runVelocity(speeds);
  }

  /**
   * Checks if the robot is near the current target pose within the position tolerance.
   *
   * @return True if the robot is within the position tolerance of the current target
   */
  public boolean isNearTarget() {
    return isNearTarget(this.currentTargetPose);
  }

  /**
   * Checks if the robot is near the specified target pose within the position tolerance.
   *
   * @param targetPose The target pose to check against
   * @return True if the robot is within the position tolerance of the target
   */
  public boolean isNearTarget(Pose2d targetPose) {
    Pose2d currentPose = swerveDriveSubsystem.getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    return distance < positionTolerance;
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
        .until(this::isNearTarget);
  }

  /**
   * Creates a command that executes the precision alignment to a fixed pose.
   *
   * @param targetPose The fixed target pose to align to
   * @return A command that will execute the precision alignment
   */
  public Command getCommand(Pose2d targetPose) {
    setCurrentTargetPose(targetPose);
    return getCommand(() -> targetPose);
  }
}
