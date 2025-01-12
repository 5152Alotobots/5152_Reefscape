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
package frc.alotobots.reefscape.commands.scoring.reef.alignment;

import static frc.alotobots.reefscape.commands.scoring.reef.alignment.constants.AlignToReefBranchConstants.ALIGNMENT_RADIUS;
import static frc.alotobots.reefscape.commands.scoring.reef.alignment.constants.AlignToReefBranchConstants.POSITION_TOLERANCE;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.reefscape.FieldConstants;
import frc.alotobots.reefscape.FieldConstants.Level;
import frc.alotobots.reefscape.FieldConstants.ReefBranch;
import frc.alotobots.reefscape.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class AlignToReefBranch extends Command {
  private final SwerveDriveSubsystem swerveDrive;
  private final ReefBranch targetBranch;
  private final Level targetLevel;
  private final PPHolonomicDriveController controller;
  private final PathPlannerTrajectoryState targetTrajectoryState;
  private Pose2d targetPose;

  public AlignToReefBranch(
      SwerveDriveSubsystem swerveDrive, ReefBranch targetBranch, Level targetLevel) {
    this.swerveDrive = swerveDrive;
    this.targetBranch = targetBranch;
    this.targetLevel = targetLevel;
    this.controller = Constants.tunerConstants.getHolonomicDriveController();
    this.targetTrajectoryState = new PathPlannerTrajectoryState();

    addRequirements(swerveDrive);
  }

  /**
   * Calculates the target pose based on branch and level. This includes both position and robot
   * orientation taking into account the alliance color.
   */
  private Pose2d calculateTargetPose() {
    // Get the branch end pose based on alliance color
    Pose2d branchEndPose = getBranchPose();

    // Calculate approach point offset from branch end
    double approachDistance = RobotConstants.ROBOT_LENGTH / 2.0;
    Translation2d backwardOffset =
        new Translation2d(
            -approachDistance * Math.cos(branchEndPose.getRotation().getRadians()),
            -approachDistance * Math.sin(branchEndPose.getRotation().getRadians()));

    // Create final target pose with position and orientation
    Translation2d targetPosition = branchEndPose.getTranslation().plus(backwardOffset);
    return new Pose2d(targetPosition, branchEndPose.getRotation());
  }

  /** Gets the branch end pose based on alliance color */
  private Pose2d getBranchPose() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    return (isRed
            ? FieldConstants.getBranchRed(targetBranch, targetLevel)
            : FieldConstants.getBranchBlue(targetBranch, targetLevel))
        .toPose2d();
  }

  @Override
  public void initialize() {
    targetPose = calculateTargetPose();
    targetTrajectoryState.pose = targetPose;
    Logger.recordOutput("Commands/SimpleAlign/Status", "Initialized");
    Logger.recordOutput("Commands/SimpleAlign/TargetPose", targetPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerveDrive.getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    Logger.recordOutput("Commands/SimpleAlign/Distance", distance);
    Logger.recordOutput("Commands/SimpleAlign/InRadius", distance <= ALIGNMENT_RADIUS);

    // Only execute alignment if within radius
    if (distance <= ALIGNMENT_RADIUS) {
      Logger.recordOutput("Commands/SimpleAlign/Status", "Aligning");

      // Use PathPlanner's holonomic controller for consistent control
      targetTrajectoryState.pose = targetPose;
      ChassisSpeeds speeds =
          controller.calculateRobotRelativeSpeeds(currentPose, targetTrajectoryState);

      swerveDrive.runVelocity(speeds);
    } else {
      swerveDrive.stop();
      Logger.recordOutput("Commands/SimpleAlign/Status", "Outside Radius");
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
    Logger.recordOutput("Commands/SimpleAlign/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    double distance =
        swerveDrive.getPose().getTranslation().getDistance(targetPose.getTranslation());

    // Finish if we're either:
    // 1. Outside the alignment radius
    // 2. Close enough to target
    return distance > ALIGNMENT_RADIUS || distance < POSITION_TOLERANCE;
  }
}
