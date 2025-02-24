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

import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PathfindThenFollowPathWithDriveOverride extends Command {
  private final PathPlannerPath goalPath;
  private final PathConstraints constraints;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
  private final PathFollowingController controller;
  private final RobotConfig robotConfig;
  private final BooleanSupplier shouldFlipPath;

  private final Supplier<ChassisSpeeds> driverInput;
  private final double inputDeadband;
  private final boolean smoothTransition;
  private final double replanWaitTime;

  private PathfindThenFollowPath currentPath;
  private boolean isDriverControlled;
  private final Timer lastInputTimer = new Timer();

  private final Pose2d targetPose;

  public PathfindThenFollowPathWithDriveOverride(
      PathPlannerPath goalPath,
      PathConstraints constraints,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Supplier<ChassisSpeeds> driverInput,
      double inputDeadband,
      boolean smoothTransition,
      double replanWaitTime,
      Subsystem... requirements) {

    this.goalPath = goalPath;
    this.constraints = constraints;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.output = output;
    this.controller = controller;
    this.robotConfig = robotConfig;
    this.shouldFlipPath = shouldFlipPath;

    this.driverInput = driverInput;
    this.inputDeadband = inputDeadband;
    this.smoothTransition = smoothTransition;
    this.replanWaitTime = replanWaitTime;

    // Store target pose for telemetry and replanning
    this.targetPose =
        new Pose2d(
            goalPath.getPoint(goalPath.numPoints() - 1).position,
            goalPath.getGoalEndState().rotation());

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    isDriverControlled = false;
    lastInputTimer.restart();
    startNewPathfinding();

    Logger.recordOutput("PathfindingOverride/TargetPose", targetPose);
  }

  @Override
  public void execute() {
    boolean hasDriverInput = hasDriverInput();
    Logger.recordOutput("PathfindingOverride/HasDriverInput", hasDriverInput);

    if (hasDriverInput) {
      lastInputTimer.reset();
    }

    if (hasDriverInput && !isDriverControlled) {
      // Transition to driver control
      if (!smoothTransition) {
        CommandScheduler.getInstance().cancel(this);
        return;
      }

      if (currentPath != null) {
        currentPath.end(true);
        currentPath = null;
      }

      isDriverControlled = true;
      Logger.recordOutput("PathfindingOverride/State", "DriverControl");
    } else if (!hasDriverInput && isDriverControlled && lastInputTimer.hasElapsed(replanWaitTime)) {
      // Transition back to autonomous after waiting
      isDriverControlled = false;
      startNewPathfinding();
      Logger.recordOutput("PathfindingOverride/State", "Autonomous");
    }

    if (isDriverControlled) {
      // Use driver input directly
      output.accept(driverInput.get(), DriveFeedforwards.zeros(robotConfig.numModules));
    } else if (currentPath != null) {
      // Execute current path and log progress
      currentPath.execute();

      Pose2d currentPose = poseSupplier.get();
      Logger.recordOutput(
          "PathfindingOverride/DistanceToTarget",
          currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    }
  }

  private boolean hasDriverInput() {
    ChassisSpeeds input = driverInput.get();
    return Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) > inputDeadband
        || Math.abs(input.omegaRadiansPerSecond) > inputDeadband;
  }

  private void startNewPathfinding() {
    if (goalPath.numPoints() < 2) {
      Logger.recordOutput("PathfindingOverride/Error", "Invalid path - too few points");
      return;
    }

    currentPath =
        new PathfindThenFollowPath(
            goalPath,
            constraints,
            poseSupplier,
            speedsSupplier,
            output,
            controller,
            robotConfig,
            shouldFlipPath);
    currentPath.initialize();

    Logger.recordOutput("PathfindingOverride/StartedNewPath", true);
  }

  @Override
  public void end(boolean interrupted) {
    if (currentPath != null) {
      currentPath.end(interrupted);
    }

    if (!smoothTransition) {
      output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
    }

    Logger.recordOutput("PathfindingOverride/State", "Ended");
  }

  @Override
  public boolean isFinished() {
    if (!smoothTransition && hasDriverInput()) {
      return true;
    }

    return currentPath != null && currentPath.isFinished();
  }

  public boolean isPathfinding() {
    return currentPath != null && !isDriverControlled;
  }
}
