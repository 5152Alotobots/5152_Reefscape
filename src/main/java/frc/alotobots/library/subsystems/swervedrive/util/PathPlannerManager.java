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
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.util.LocalADStarAK;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
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
        driveSubsystem::runVelocityWithSetpointGen,
        Constants.tunerConstants.getHolonomicDriveController(),
        Constants.tunerConstants.getPathPlannerConfig(),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        driveSubsystem);

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
              "Pathplanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Pathplanner/TrajectorySetpoint", targetPose);
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
   * @param pathName The path name to follow after pathfinding to it
   * @return The command to be scheduled
   */
  public Command getPathfindThenFollowPathCommand(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Command pathCommand =
          AutoBuilder.pathfindThenFollowPath(
              path, Constants.tunerConstants.getPathfindingConstraints());
      return pathCommand;
    } catch (Exception e) {
      String errorMessage = "Failed to load path: " + pathName;
      return new PrintCommand(errorMessage + " Not following path!");
    }
  }

  /**
   * Creates a command to pathfind to and then follow a path with driver override capability.
   *
   * @param pathName The path name to follow after pathfinding to it
   * @param driverInput Supplier for driver's requested chassis speeds
   * @param smoothTransition Whether to smoothly transition between autonomous and driver control
   * @return The command to be scheduled
   */
  public Command getPathfindThenFollowPathCommandWithOverride(
      String pathName, Supplier<ChassisSpeeds> driverInput, boolean smoothTransition) {

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      var constraints = Constants.tunerConstants.getPathfindingConstraints();

      return new PathfindThenFollowPathWithDriveOverride(
          path,
          constraints,
          driveSubsystem::getPose,
          driveSubsystem::getChassisSpeeds,
          (speeds, ff) -> driveSubsystem.runVelocityWithSetpointGen(speeds),
          Constants.tunerConstants.getHolonomicDriveController(),
          Constants.tunerConstants.getPathPlannerConfig(),
          AutoBuilder::shouldFlip,
          driverInput,
          0.1, // deadband - now applies to chassis speeds magnitudes
          smoothTransition,
          0.2,
          driveSubsystem);
    } catch (Exception e) {
      String errorMessage = "Failed to load path: " + pathName;
      Logger.recordOutput("PathPlanner/Error", errorMessage);
      return new PrintCommand(errorMessage + " Not following path!");
    }
  }

  /**
   * Gets the starting pose from a PathPlanner path.
   *
   * @param pathName The name of the path to load
   * @return Optional containing the starting pose, or empty if path cannot be loaded
   */
  public Optional<Pose2d> getPathStartPose(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Optional<PathPlannerTrajectory> expectedTrajectory =
          path.getIdealTrajectory(Constants.tunerConstants.getPathPlannerConfig());
      if (expectedTrajectory.isPresent()) {
        PathPlannerTrajectory trajectory = expectedTrajectory.get();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
          return Optional.of(FlippingUtil.flipFieldPose(trajectory.getInitialState().pose));
        }
        return Optional.of(trajectory.getInitialState().pose);
      }
      return Optional.empty();
    } catch (Exception e) {
      System.err.println("Failed to load path start pose: " + pathName);
      e.printStackTrace();
      return Optional.empty();
    }
  }

  /**
   * Gets the final target pose from a PathPlanner path.
   *
   * @param pathName The name of the path to load
   * @return Optional containing the final pose, or empty if path cannot be loaded
   */
  public Optional<Pose2d> getPathEndPose(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Optional<PathPlannerTrajectory> expectedTrajectory =
          path.getIdealTrajectory(Constants.tunerConstants.getPathPlannerConfig());
      if (expectedTrajectory.isPresent()) {
        PathPlannerTrajectory trajectory = expectedTrajectory.get();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
          return Optional.of(FlippingUtil.flipFieldPose(trajectory.getEndState().pose));
        }
        return Optional.of(trajectory.getEndState().pose);
      }
      return Optional.empty();
    } catch (Exception e) {
      System.err.println("Failed to load path end pose: " + pathName);
      e.printStackTrace();
      return Optional.empty();
    }
  }

  /**
   * Whether the current robot pose is near the end of a path
   *
   * @param pathName The path name to get the end pose from
   * @param translationalTolerance The translational tolerance in meters
   * @param rotationalTolerance The rotational tolerance in radians
   * @return true if near the end of a path
   */
  public boolean nearEndOfPath(
      String pathName, double translationalTolerance, double rotationalTolerance) {
    if (getPathEndPose(pathName).isPresent()) {
      Pose2d current = driveSubsystem.getPose();
      Pose2d target = getPathEndPose(pathName).get();

      // Check translational distance using hypot (handles X and Y together)
      double translationalDiff = current.getTranslation().getDistance(target.getTranslation());

      // Get absolute rotation difference and normalize to [-pi, pi]
      double rotationalDiff = current.getRotation().minus(target.getRotation()).getRadians();

      return translationalDiff <= translationalTolerance && rotationalDiff <= rotationalTolerance;
    }
    return false;
  }

  /**
   * Whether the current robot pose is near the start of a path
   *
   * @param pathName The path name to get the end pose from
   * @param translationalTolerance The translational tolerance in meters
   * @param rotationalTolerance The rotational tolerance in radians
   * @return true if near the start of a path
   */
  public boolean nearStartOfPath(
      String pathName, double translationalTolerance, double rotationalTolerance) {
    if (getPathStartPose(pathName).isPresent()) {
      Pose2d current = driveSubsystem.getPose();
      Pose2d target = getPathStartPose(pathName).get();

      // Check translational distance using hypot (handles X and Y together)
      double translationalDiff = current.getTranslation().getDistance(target.getTranslation());

      // Get absolute rotation difference and normalize to [-pi, pi]
      double rotationalDiff =
          Math.abs(current.getRotation().minus(target.getRotation()).getRadians());
      if (rotationalDiff > Math.PI) {
        rotationalDiff = 2 * Math.PI - rotationalDiff;
      }

      return translationalDiff <= translationalTolerance && rotationalDiff <= rotationalTolerance;
    }
    return false;
  }

  /**
   * Gets the trajectory points for a given path name.
   *
   * @param pathName The name of the path to get the trajectory for
   * @return Optional containing List of Pose2d points if path exists, empty Optional otherwise
   */
  public Optional<List<Pose2d>> getPathTrajectory(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      if (path == null) {
        return Optional.empty();
      }

      // Get the states at fixed intervals along the path
      List<Pose2d> trajectoryPoses = path.getPathPoses();
      return Optional.of(trajectoryPoses);
    } catch (Exception e) {
      return Optional.empty();
    }
  }
}
