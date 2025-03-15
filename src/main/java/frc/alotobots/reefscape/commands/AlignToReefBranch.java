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
package frc.alotobots.reefscape.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DrivePrecisionAlign;
import frc.alotobots.reefscape.FieldConstants;

/** Command that aligns the robot to the closest reef branch on the specified side. */
public class AlignToReefBranch extends Command {

  public enum ReefBranchSide {
    LEFT,
    RIGHT
  }

  private final Distance ALIGN_ALLOW_DISTANCE = Meters.of(0.1); // Allowable distance for alignment
  private final Angle HEADING_MATCH_REQUIREMENT = Degrees.of(30); // Allowable heading difference

  private final ReefBranchSide side;
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final DrivePrecisionAlign request;
  private final FieldConstants.Level branchLevel;

  private DriverStation.Alliance alliance;
  private Pose2d targetPose;

  /**
   * Creates a new AlignToReefBranch command with the specified branch level.
   *
   * @param swerveDriveSubsystem The swerve drive subsystem
   * @param side The side of the reef branch to align to (LEFT or RIGHT)
   * @param branchLevel The level of the reef branch to target
   */
  public AlignToReefBranch(
      SwerveDriveSubsystem swerveDriveSubsystem,
      ReefBranchSide side,
      FieldConstants.Level branchLevel) {
    this.side = side;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.branchLevel = branchLevel;
    this.request = new DrivePrecisionAlign(swerveDriveSubsystem);
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveDriveSubsystem.getPose();

    // Check if alliance information is available
    if (DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();

      // Get the appropriate reef branch pose based on alliance and side
      Pose3d targetBranchPose = getTargetBranchPose(currentPose);

      // Convert to Pose2d for alignment
      targetPose = targetBranchPose.toPose2d();

      // Check if we're already within alignment tolerance
      if (isAlreadyAligned(currentPose, targetBranchPose)) {
        return;
      }
    } else {
      // Cannot align without alliance information
      cancel();
    }
  }

  /**
   * Gets the target branch pose based on the current pose, alliance, and selected side.
   *
   * @param currentPose The current robot pose
   * @return The target branch pose in 3D space
   */
  private Pose3d getTargetBranchPose(Pose2d currentPose) {
    if (alliance == DriverStation.Alliance.Red) {
      return (side == ReefBranchSide.LEFT)
          ? FieldConstants.getClosestLeftBranchRed(currentPose, branchLevel)
          : FieldConstants.getClosestRightBranchRed(currentPose, branchLevel);
    } else {
      return (side == ReefBranchSide.LEFT)
          ? FieldConstants.getClosestLeftBranchBlue(currentPose, branchLevel)
          : FieldConstants.getClosestRightBranchBlue(currentPose, branchLevel);
    }
  }

  /**
   * Checks if the robot is already aligned with the target branch.
   *
   * @param currentPose The current robot pose
   * @param targetBranchPose The target branch pose
   * @return True if already aligned within tolerance
   */
  private boolean isAlreadyAligned(Pose2d currentPose, Pose3d targetBranchPose) {
    double distanceToTarget =
        currentPose
            .getTranslation()
            .getDistance(targetBranchPose.getTranslation().toTranslation2d());

    double headingDifference =
        Math.abs(currentPose.getRotation().getDegrees() - targetBranchPose.getRotation().getZ());

    return (distanceToTarget <= ALIGN_ALLOW_DISTANCE.in(Meters))
        && (headingDifference <= HEADING_MATCH_REQUIREMENT.in(Degrees));
  }

  @Override
  public void execute() {
    // Apply the precision alignment request with the target pose
    request.applyRequest(() -> targetPose);
  }

  @Override
  public boolean isFinished() {
    // Command is finished when we're near the target
    return request.isNearTarget();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    swerveDriveSubsystem.stop();
  }
}
