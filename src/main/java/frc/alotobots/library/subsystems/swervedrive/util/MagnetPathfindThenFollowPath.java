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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class MagnetPathfindThenFollowPath extends Command {
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

  private PathfindThenFollowPath currentPath;
  private ChassisSpeeds lastPathfindingSpeeds = new ChassisSpeeds();

  @Getter private final Pose2d targetPose;

  public MagnetPathfindThenFollowPath(
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

    // Store target pose for telemetry
    this.targetPose =
        new Pose2d(
            goalPath.getPoint(goalPath.numPoints() - 1).position,
            goalPath.getGoalEndState().rotation());

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    startNewPathfinding();
    Logger.recordOutput("MagnetPathfind/TargetPose", targetPose);
  }

  @Override
  public void execute() {
    // Get current inputs
    ChassisSpeeds driverSpeeds = driverInput.get();
    boolean hasXInput = Math.abs(driverSpeeds.vxMetersPerSecond) > inputDeadband;
    boolean hasYInput = Math.abs(driverSpeeds.vyMetersPerSecond) > inputDeadband;
    boolean hasRotInput = Math.abs(driverSpeeds.omegaRadiansPerSecond) > inputDeadband;

    Logger.recordOutput("MagnetPathfind/HasXInput", hasXInput);
    Logger.recordOutput("MagnetPathfind/HasYInput", hasYInput);
    Logger.recordOutput("MagnetPathfind/HasRotInput", hasRotInput);

    // Always run pathfinding
    if (currentPath != null) {
      currentPath.execute();
      lastPathfindingSpeeds = speedsSupplier.get();

      // Log pathfinding progress
      Pose2d currentPose = poseSupplier.get();
      Logger.recordOutput(
          "MagnetPathfind/DistanceToTarget",
          currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    }

    // Blend pathfinding and driver inputs
    ChassisSpeeds blendedSpeeds =
        new ChassisSpeeds(
            hasXInput ? driverSpeeds.vxMetersPerSecond : lastPathfindingSpeeds.vxMetersPerSecond,
            hasYInput ? driverSpeeds.vyMetersPerSecond : lastPathfindingSpeeds.vyMetersPerSecond,
            hasRotInput
                ? driverSpeeds.omegaRadiansPerSecond
                : lastPathfindingSpeeds.omegaRadiansPerSecond);

    // Apply the blended speeds
    output.accept(blendedSpeeds, DriveFeedforwards.zeros(robotConfig.numModules));

    Logger.recordOutput(
        "MagnetPathfind/BlendedSpeeds",
        new double[] {
          blendedSpeeds.vxMetersPerSecond,
          blendedSpeeds.vyMetersPerSecond,
          blendedSpeeds.omegaRadiansPerSecond
        });
  }

  private void startNewPathfinding() {
    if (goalPath.numPoints() < 2) {
      Logger.recordOutput("MagnetPathfind/Error", "Invalid path - too few points");
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

    Logger.recordOutput("MagnetPathfind/StartedNewPath", true);
  }

  @Override
  public void end(boolean interrupted) {
    if (currentPath != null) {
      currentPath.end(interrupted);
    }

    output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
    Logger.recordOutput("MagnetPathfind/State", "Ended");
  }

  @Override
  public boolean isFinished() {
    return currentPath != null && currentPath.isFinished();
  }

  public boolean isPathfinding() {
    return currentPath != null;
  }
}
