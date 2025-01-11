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
package frc.alotobots.library.subsystems.swervedrive.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.AutoNamedCommands;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.reefscape.FieldConstants;
import frc.alotobots.util.LocalADStarAK;
import org.littletonrobotics.junction.Logger;

/**
 * Manages PathPlanner integration for autonomous path following and path finding. Abstracts path
 * planning functionality from the SwerveDriveSubsystem.
 */
public class PathPlannerManager {
  private final SwerveDriveSubsystem driveSubsystem;

  /**
   * Creates a new PathPlannerManager.
   *
   * @param driveSubsystem The swerve drive subsystem to control
   */
  public PathPlannerManager(SwerveDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    configurePathPlanner();
  }

  /** Configures PathPlanner with necessary callbacks and settings. */
  private void configurePathPlanner() {
    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        driveSubsystem::getPose,
        driveSubsystem::setPose,
        driveSubsystem::getChassisSpeeds,
        driveSubsystem::runVelocity,
        Constants.tunerConstants.getHolonomicDriveController(),
        Constants.tunerConstants.getPathPlannerConfig(),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        driveSubsystem);

    // Setup named commands for autonomous routines
    AutoNamedCommands.setupNamedCommands();

    // Configure pathfinding
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Setup logging callbacks
    configureLogging();
  }

  /** Configures PathPlanner logging callbacks. */
  private void configureLogging() {
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  /**
   * Creates a pathfinding command to the specified pose.
   *
   * @param target Target pose
   * @param velocity Target velocity
   * @return Pathfinding command
   */
  public Command getPathFinderCommand(Pose2d target, LinearVelocity velocity) {
    return AutoBuilder.pathfindToPose(
        target, Constants.tunerConstants.getPathfindingConstraints(), velocity);
  }

  /**
   * Gets the path name for approaching a specific reef branch at a given level.
   *
   * @param branch Target branch
   * @param level Target level
   * @return Path name for the approach
   */
  private String getReefPathName(FieldConstants.ReefBranch branch, FieldConstants.Level level) {
    // Convention: "BranchApproach_[Branch]_[Level]"
    return String.format("BranchApproach_%s_%s", branch, level);
  }

  /**
   * Creates a command that will pathfind to and then follow a pre-made path to the target branch.
   *
   * @param branch Target branch
   * @param level Target level
   * @return Combined pathfind and follow command, or null if path loading fails
   */
  public Command getReefBranchCommand(
      FieldConstants.ReefBranch branch, FieldConstants.Level level) {
    String pathName = getReefPathName(branch, level);
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.pathfindThenFollowPath(
          path, Constants.tunerConstants.getPathfindingConstraints());
    } catch (Exception e) {
      // Log the error
      System.err.println("Failed to load path: " + pathName);
      return null;
    }
  }

  /**
   * Gets maximum linear speed capability.
   *
   * @return Maximum speed in meters per second
   */
  public double getMaxLinearSpeedMetersPerSec() {
    return driveSubsystem.getMaxLinearSpeedMetersPerSec();
  }

  /**
   * Gets maximum angular speed capability.
   *
   * @return Maximum angular speed in radians per second
   */
  public double getMaxAngularSpeedRadPerSec() {
    return driveSubsystem.getMaxAngularSpeedRadPerSec();
  }
}
