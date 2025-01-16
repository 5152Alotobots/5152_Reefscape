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
package frc.alotobots.reefscape.subsystems.autocycle.reef.commands;

import static frc.alotobots.reefscape.subsystems.autocycle.reef.constants.AutoCycleReefConstants.ALIGNMENT_RADIUS;
import static frc.alotobots.reefscape.subsystems.autocycle.reef.constants.AutoCycleReefConstants.POSITION_TOLERANCE;

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

/**
 * A command that aligns the robot to a specific reef branch at a given level. This command uses
 * PathPlanner's holonomic drive controller to precisely position the robot relative to the target
 * branch, taking into account the alliance color and maintaining proper orientation throughout the
 * alignment process.
 */
public class FinalAlignToReefBranch extends Command {
  /** The swerve drive subsystem used for robot movement */
  private final SwerveDriveSubsystem swerveDrive;

  /** The target reef branch to align with */
  private final ReefBranch targetBranch;

  /** The vertical level of the target branch */
  private final Level targetLevel;

  /** The PathPlanner controller for holonomic movement */
  private final PPHolonomicDriveController controller;

  /** The trajectory state representing the target position and orientation */
  private final PathPlannerTrajectoryState targetTrajectoryState;

  /** The final target pose for the robot */
  private Pose2d targetPose;

  /**
   * Creates a new AlignToReefBranch command.
   *
   * @param swerveDrive The swerve drive subsystem used for movement
   * @param targetBranch The reef branch to align with
   * @param targetLevel The vertical level of the target branch
   */
  public FinalAlignToReefBranch(
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
   *
   * @return The calculated target Pose2d for the robot
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

  /**
   * Gets the branch end pose based on alliance color.
   *
   * @return The Pose2d of the branch end for the current alliance
   */
  private Pose2d getBranchPose() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    return (isRed
            ? FieldConstants.getBranchRed(targetBranch, targetLevel)
            : FieldConstants.getBranchBlue(targetBranch, targetLevel))
        .toPose2d();
  }

  /**
   * Called when the command is initially scheduled. Calculates the initial target pose and
   * initializes logging.
   */
  @Override
  public void initialize() {
    targetPose = calculateTargetPose();
    targetTrajectoryState.pose = targetPose;
    Logger.recordOutput("Commands/AlignToReefBranch/Status", "Initialized");
    Logger.recordOutput("Commands/AlignToReefBranch/TargetPose", targetPose);
  }

  /**
   * Called repeatedly when this command is scheduled to run. Handles the alignment process when the
   * robot is within the alignment radius.
   */
  @Override
  public void execute() {
    Pose2d currentPose = swerveDrive.getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    Logger.recordOutput("Commands/AlignToReefBranch/Distance", distance);
    Logger.recordOutput("Commands/AlignToReefBranch/InRadius", distance <= ALIGNMENT_RADIUS);

    // Only execute alignment if within radius
    if (distance <= ALIGNMENT_RADIUS) {
      Logger.recordOutput("Commands/AlignToReefBranch/Status", "Aligning");

      // Use PathPlanner's holonomic controller for consistent control
      targetTrajectoryState.pose = targetPose;
      ChassisSpeeds speeds =
          controller.calculateRobotRelativeSpeeds(currentPose, targetTrajectoryState);

      swerveDrive.runVelocity(speeds);
    } else {
      swerveDrive.stop();
      Logger.recordOutput("Commands/AlignToReefBranch/Status", "Outside Radius");
    }
  }

  /**
   * Called once when the command ends or is interrupted. Stops the robot's movement and logs the
   * final status.
   *
   * @param interrupted Whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
    Logger.recordOutput(
        "Commands/AlignToReefBranch/Status", interrupted ? "Interrupted" : "Completed");
  }

  /**
   * Returns whether the command should end. The command ends when the robot is either outside the
   * alignment radius or has reached the target position within tolerance.
   *
   * @return true if the command should end, false otherwise
   */
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
