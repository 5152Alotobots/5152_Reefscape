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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DrivePrecisionAlign;
import frc.alotobots.reefscape.FieldConstants;

/** Command that aligns the robot to the closest reef branch on the specified side. */
public class AlignToReefBranch extends Command {

  private final Distance ALIGN_ALLOW_DISTANCE = Meters.of(0.25); // Allowable distance for alignment
  private final Angle HEADING_MATCH_REQUIREMENT = Degrees.of(30); // Allowable heading difference
  private final Transform2d ALIGN_OFFSET_TRANSFORM =
      new Transform2d(
          new Translation2d(-0.58, 0),
          Rotation2d.kZero); // Distance from the reef branch to align at

  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final DrivePrecisionAlign request;
  private final FieldConstants.BranchType branchType;
  private DriverStation.Alliance alliance;
  private Pose2d targetPose;

  /**
   * Creates a new AlignToReefBranch command.
   *
   * @param swerveDriveSubsystem The swerve drive subsystem
   * @param branchType The type of the reef branch to align to (LEFT, RIGHT, ANY)
   */
  public AlignToReefBranch(
      SwerveDriveSubsystem swerveDriveSubsystem, FieldConstants.BranchType branchType) {
    this.branchType = branchType;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.request = new DrivePrecisionAlign(swerveDriveSubsystem, 0.01);
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveDriveSubsystem.getPose();

    // Check if alliance information is available
    if (DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();

      // Get the appropriate reef branch pose based on alliance and side
      Pose2d targetBranchPose =
          FieldConstants.BranchPositions.getClosestBranch(currentPose, alliance, branchType);

      targetPose = targetBranchPose.transformBy(ALIGN_OFFSET_TRANSFORM);
      // Put on dash
      swerveDriveSubsystem.logAutoAlignTargetPose(targetPose);

      // Check if we're in the required zone for aligning
      if (!shouldAlign(currentPose, targetPose)) {
        cancel();
      }
    } else {
      // Cannot align without alliance information
      cancel();
    }
  }

  /**
   * Checks if the robot is already aligned with the target branch.
   *
   * @param currentPose The current robot pose
   * @param targetBranchPose The target branch pose
   * @return True if already aligned within tolerance
   */
  private boolean shouldAlign(Pose2d currentPose, Pose2d targetBranchPose) {
    double distanceToTarget =
        currentPose.getTranslation().getDistance(targetBranchPose.getTranslation());

    double headingDifference =
        Math.abs(
            currentPose.getRotation().getDegrees() - targetBranchPose.getRotation().getDegrees());

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
    swerveDriveSubsystem.stopWithX();
  }
}
