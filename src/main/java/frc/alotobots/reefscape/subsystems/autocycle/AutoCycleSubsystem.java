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
package frc.alotobots.reefscape.subsystems.autocycle;

import static frc.alotobots.reefscape.subsystems.autocycle.constants.AutoCycleConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DrivePrecisionAlign;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.reefscape.FieldConstants;
import java.util.List;
import java.util.Optional;

import frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState;
import lombok.Getter;

/**
 * Subsystem that manages the automatic cycling of reef branches and levels for autonomous
 * navigation. This subsystem enables automated movement patterns during matches by allowing
 * selection and cycling through different branches and levels of the reef, as well as
 * coral station positions for game piece collection.
 */
public class AutoCycleSubsystem extends SubsystemBase {
  /**
   * The current state of the subsystem.
   * Contains all selection and pathfinding status information.
   */
  @Getter private final AutoCycleState state;

  /**
   * The pathplanner manager instance to use for pathfinding.
   * Handles path generation and following.
   */
  private final PathPlannerManager pathPlannerManager;

  /**
   * The swerve drive instance to use.
   * Controls robot movement and positioning.
   */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /**
   * Creates a new AutoCycleReefSubsystem with default branch and level selections.
   *
   * @param pathPlannerManager The PathPlannerManager instance to use for navigation
   * @param swerveDriveSubsystem The SwerveDriveSubsystem instance for robot movement control
   */
  public AutoCycleSubsystem(
          PathPlannerManager pathPlannerManager, SwerveDriveSubsystem swerveDriveSubsystem) {
    this.state = AutoCycleState.createDefault();
    this.pathPlannerManager = pathPlannerManager;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  /**
   * Periodic update method called by the scheduler.
   * Updates path planning and logs state information.
   */
  @Override
  public void periodic() {
    // Log all state information
    state.logState();

    // Update and log path information
    updateActivePath();
  }

  /**
   * Logs the target poses for both reef and coral station paths.
   * Records the end poses of current paths for debugging and visualization.
   */
  /**
   * Updates target pose information in logs.
   * Gets and logs the current target poses for both reef and coral station paths.
   */
  private void updateTargetPoses() {
    String selectedReefBranchPathName = state.getSelectedReefBranchPathName();
    String selectedCoralStationPathName = state.getSelectedCoralStationPathName();

    Optional<Pose2d> reefPose = pathPlannerManager.getPathEndPose(selectedReefBranchPathName);
    Optional<Pose2d> stationPose = pathPlannerManager.getPathEndPose(selectedCoralStationPathName);

    state.logTargetPoses(reefPose, stationPose);
  }

  /**
   * Updates path data in logs for the specified path.
   * Gets and logs current path information including poses and trajectory.
   *
   * @param pathName The name of the path to log data for
   */
  private void updatePathData(String pathName) {
    Optional<Pose2d> endPose = pathPlannerManager.getPathEndPose(pathName);
    Optional<List<Pose2d>> trajectory = pathPlannerManager.getPathTrajectory(pathName);

    state.logPathData(endPose, trajectory);
  }

  /**
   * Updates the active pathfinding path if necessary.
   * Checks if the target position has changed while actively pathfinding and
   * automatically updates the path to the new target.
   */
  private void updateActivePath() {
    if (!state.isPathfindingToCoralStation() && !state.isPathfindingToReefBranch()) {
      return; // No active pathfinding to update
    }

    String targetPath =
            state.isPathfindingToCoralStation()
                    ? state.getSelectedCoralStationPathName()
                    : state.getSelectedReefBranchPathName();

    // Update path if target has changed or this is initial scheduling
    if (!targetPath.equals(state.getActivePath()) || state.getActivePath().isEmpty()) {
      state.setActivePath(targetPath);
      pathPlannerManager.getPathfindThenFollowPathCommand(targetPath).schedule();

      // Log updated path data
      updatePathData(targetPath);
      updateTargetPoses();
    }
  }

  /**
   * Cycles to the next reef branch to the right.
   * Advances the selection to the next branch, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at end and wrap not allowed
   */
  public boolean cycleReefBranchRight() {
    return cycleReefBranchRight(true);
  }

  /**
   * Cycles to the next reef branch to the right.
   *
   * @param allowWrap If true, wraps around to first branch when at last branch
   * @return true if cycling was successful, false if at end and wrap not allowed
   */
  public boolean cycleReefBranchRight(boolean allowWrap) {
    int nextOrdinal = state.getReefBranch().ordinal() + 1;
    if (nextOrdinal >= FieldConstants.ReefBranch.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    state.setReefBranch(FieldConstants.ReefBranch.values()[nextOrdinal]);
    return true;
  }

  /**
   * Cycles to the next reef branch to the left.
   * Advances the selection to the previous branch, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at start and wrap not allowed
   */
  public boolean cycleReefBranchLeft() {
    return cycleReefBranchLeft(true);
  }

  /**
   * Cycles to the next reef branch to the left.
   *
   * @param allowWrap If true, wraps around to last branch when at first branch
   * @return true if cycling was successful, false if at start and wrap not allowed
   */
  public boolean cycleReefBranchLeft(boolean allowWrap) {
    int nextOrdinal = state.getReefBranch().ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.ReefBranch.values().length - 1;
    }
    state.setReefBranch(FieldConstants.ReefBranch.values()[nextOrdinal]);
    return true;
  }

  /**
   * Cycles to the next level up.
   * Advances the selection to a higher level, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at highest and wrap not allowed
   */
  public boolean cycleReefLevelUp() {
    return cycleReefLevelUp(false);
  }

  /**
   * Cycles to the next level up.
   *
   * @param allowWrap If true, wraps around to lowest level when at highest level
   * @return true if cycling was successful, false if at highest and wrap not allowed
   */
  public boolean cycleReefLevelUp(boolean allowWrap) {
    int nextOrdinal = state.getReefLevel().ordinal() + 1;
    if (nextOrdinal >= FieldConstants.Level.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    state.setReefLevel(FieldConstants.Level.values()[nextOrdinal]);
    return true;
  }

  /**
   * Cycles to the next level down.
   * Advances the selection to a lower level, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at lowest and wrap not allowed
   */
  public boolean cycleReefLevelDown() {
    return cycleReefLevelDown(false);
  }

  /**
   * Cycles to the next level down.
   *
   * @param allowWrap If true, wraps around to highest level when at lowest level
   * @return true if cycling was successful, false if at lowest and wrap not allowed
   */
  public boolean cycleReefLevelDown(boolean allowWrap) {
    int nextOrdinal = state.getReefLevel().ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.Level.values().length - 1;
    }
    state.setReefLevel(FieldConstants.Level.values()[nextOrdinal]);
    return true;
  }

  /**
   * Cycles to the next coral station side to the right.
   * Advances the selection to the next side, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at end and wrap not allowed
   */
  public boolean cycleCoralStationSideRight() {
    return cycleCoralStationSideRight(false);
  }

  /**
   * Cycles to the next coral station side to the right.
   *
   * @param allowWrap If true, wraps around from RIGHT to LEFT
   * @return true if cycling was successful, false if at end and wrap not allowed
   */
  public boolean cycleCoralStationSideRight(boolean allowWrap) {
    int nextOrdinal = state.getCoralStationSide().ordinal() + 1;
    if (nextOrdinal >= FieldConstants.CoralStationSide.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    state.setCoralStationSide(FieldConstants.CoralStationSide.values()[nextOrdinal]);
    return true;
  }

  /**
   * Cycles to the next coral station side to the left.
   * Advances the selection to the previous side, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at start and wrap not allowed
   */
  public boolean cycleCoralStationSideLeft() {
    return cycleCoralStationSideLeft(false);
  }

  /**
   * Cycles to the next coral station side to the left.
   *
   * @param allowWrap If true, wraps around from LEFT to RIGHT
   * @return true if cycling was successful, false if at start and wrap not allowed
   */
  public boolean cycleCoralStationSideLeft(boolean allowWrap) {
    int nextOrdinal = state.getCoralStationSide().ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.CoralStationSide.values().length - 1;
    }
    state.setCoralStationSide(FieldConstants.CoralStationSide.values()[nextOrdinal]);
    return true;
  }

  /**
   * Cycles to the next pickup position to the right.
   * Advances the selection to the next position, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at end and wrap not allowed
   */
  public boolean cycleCoralStationPickupPositionRight() {
    return cycleCoralStationPickupPositionRight(false);
  }

  /**
   * Cycles to the next pickup position to the right.
   *
   * @param allowWrap If true, wraps around from P3 back to P1
   * @return true if cycling was successful, false if at end and wrap not allowed
   */
  public boolean cycleCoralStationPickupPositionRight(boolean allowWrap) {
    int nextOrdinal = state.getCoralStationPickupPosition().ordinal() + 1;
    if (nextOrdinal >= FieldConstants.CoralStationPickupPosition.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    state.setCoralStationPickupPosition(
            FieldConstants.CoralStationPickupPosition.values()[nextOrdinal]);
    return true;
  }

  /**
   * Cycles to the next pickup position to the left.
   * Advances the selection to the previous position, optionally wrapping around.
   *
   * @return true if cycling was successful, false if at start and wrap not allowed
   */
  public boolean cycleCoralStationPickupPositionLeft() {
    return cycleCoralStationPickupPositionLeft(false);
  }

  /**
   * Cycles to the next pickup position to the left.
   *
   * @param allowWrap If true, wraps around from P1 back to P3
   * @return true if cycling was successful, false if at end and wrap not allowed
   */
  public boolean cycleCoralStationPickupPositionLeft(boolean allowWrap) {
    int nextOrdinal = state.getCoralStationPickupPosition().ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.CoralStationPickupPosition.values().length - 1;
    }
    state.setCoralStationPickupPosition(
            FieldConstants.CoralStationPickupPosition.values()[nextOrdinal]);
    return true;
  }

  /**
   * Creates a command that initiates pathfinding to the currently selected reef branch position.
   * This command will either start pathfinding or switch to precision alignment based on
   * the robot's current position relative to the target.
   *
   * @return A command that when executed will start pathfinding to the selected reef branch
   */
  public Command pathfindToSelectedReefBranchPathName() {
    return Commands.either(
            this.defer(
                    () -> {
                      Optional<Pose2d> targetPose =
                              pathPlannerManager.getPathEndPose(getState().getSelectedReefBranchPathName());
                      if (targetPose.isPresent()) {
                        return new DrivePrecisionAlign(swerveDriveSubsystem).getCommand(targetPose.get());
                      }
                      return new PrintCommand("No Selected Path! No Aligning!");
                    }),
            this.runOnce(
                    () -> {
                      state.setPathfindingToReefBranch(true);
                      state.setPathfindingToCoralStation(false);
                      state.setActivePath("");
                    }),
            () ->
                    pathPlannerManager.nearEndOfPath(
                            getState().getSelectedReefBranchPathName(),
                            ALIGNMENT_TRANSLATION_TOLERANCE,
                            ALIGNMENT_ROTATION_TOLERANCE));
  }

  /**
   * Creates a command that initiates pathfinding to the currently selected coral station position.
   * This command will either start pathfinding or switch to precision alignment based on
   * the robot's current position relative to the target.
   *
   * @return A command that when executed will start pathfinding to the selected coral station
   */
  public Command pathfindToSelectedCoralStationPathName() {
    return Commands.either(
            this.defer(
                    () -> {
                      Optional<Pose2d> targetPose =
                              pathPlannerManager.getPathEndPose(getState().getSelectedCoralStationPathName());
                      if (targetPose.isPresent()) {
                        return new DrivePrecisionAlign(swerveDriveSubsystem).getCommand(targetPose.get());
                      }
                      return new PrintCommand("No Selected Path! No Aligning!");
                    }),
            this.runOnce(
                    () -> {
                      state.setPathfindingToCoralStation(true);
                      state.setPathfindingToReefBranch(false);
                      state.setActivePath("");
                    }),
            () ->
                    pathPlannerManager.nearEndOfPath(
                            getState().getSelectedCoralStationPathName(),
                            ALIGNMENT_TRANSLATION_TOLERANCE,
                            ALIGNMENT_ROTATION_TOLERANCE));
  }

  /**
   * Creates a command that stops any active pathfinding.
   * Clears all pathfinding states and stops current path following.
   *
   * @return A command that when executed will stop all active pathfinding
   */
  public Command stopPathfinding() {
    return this.runOnce(
            () -> {
              state.setPathfindingToCoralStation(false);
              state.setPathfindingToReefBranch(false);
              state.setActivePath("");
            });
  }

  /**
   * Creates a command that changes the selected reef branch to the one that is one position to the right.
   *
   * @param allowWrap If true, wraps around to first branch when at last branch
   * @return Command that will change the selected branch when executed
   */
  public Command runCycleReefBranchRight(boolean allowWrap) {
    return this.runOnce(() -> cycleReefBranchRight(allowWrap));
  }

  /**
   * Creates a command that changes the selected reef branch to the one that is one position to the left.
   *
   * @param allowWrap If true, wraps around to last branch when at first branch
   * @return Command that will change the selected branch when executed
   */
  public Command runCycleReefBranchLeft(boolean allowWrap) {
    return this.runOnce(() -> cycleReefBranchLeft(allowWrap));
  }

  /**
   * Creates a command that changes the selected reef level to the one that is one position higher.
   *
   * @param allowWrap If true, wraps around to lowest level when at highest level
   * @return Command that will change the selected level when executed
   */
  public Command runCycleReefLevelUp(boolean allowWrap) {
    return this.runOnce(() -> cycleReefLevelUp(allowWrap));
  }

  /**
   * Creates a command that changes the selected reef level to the one that is one position lower.
   *
   * @param allowWrap If true, wraps around to highest level when at lowest level
   * @return Command that will change the selected level when executed
   */
  public Command runCycleReefLevelDown(boolean allowWrap) {
    return this.runOnce(() -> cycleReefLevelDown(allowWrap));
  }

  /**
   * Creates a command that changes the selected coral station side to the one that is one position to the right.
   *
   * @param allowWrap If true, wraps around from RIGHT to LEFT
   * @return Command that will change the selected side when executed
   */
  public Command runCycleCoralStationSideRight(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationSideRight(allowWrap));
  }

  /**
   * Creates a command that changes the selected coral station side to the one that is one position to the left.
   *
   * @param allowWrap If true, wraps around from LEFT to RIGHT
   * @return Command that will change the selected side when executed
   */
  public Command runCycleCoralStationSideLeft(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationSideLeft(allowWrap));
  }

  /**
   * Creates a command that changes the selected pickup position to the one that is one position to the right.
   *
   * @param allowWrap If true, wraps around from P3 back to P1
   * @return Command that will change the selected position when executed
   */
  public Command runCycleCoralStationPickupPositionRight(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationPickupPositionRight(allowWrap));
  }

  /**
   * Creates a command that changes the selected pickup position to the one that is one position to the left.
   *
   * @param allowWrap If true, wraps around from P1 back to P3
   * @return Command that will change the selected position when executed
   */
  public Command runCycleCoralStationPickupPositionLeft(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationPickupPositionLeft(allowWrap));
  }

  /**
   * Creates a command that sets the reef branch to a specific value.
   * Directly sets the branch selection without cycling through options.
   *
   * @param branch The branch to set
   * @return Command that will set the branch when executed
   */
  public Command runSetReefBranch(FieldConstants.ReefBranch branch) {
    return this.runOnce(() -> state.setReefBranch(branch));
  }

  /**
   * Creates a command that sets the reef level to a specific value.
   * Directly sets the level selection without cycling through options.
   *
   * @param level The level to set
   * @return Command that will set the level when executed
   */
  public Command runSetReefLevel(FieldConstants.Level level) {
    return this.runOnce(() -> state.setReefLevel(level));
  }

  /**
   * Creates a command that sets the coral station side to a specific value.
   * Directly sets the side selection without cycling through options.
   *
   * @param side The side to set
   * @return Command that will set the side when executed
   */
  public Command runSetCoralStationSide(FieldConstants.CoralStationSide side) {
    return this.runOnce(() -> state.setCoralStationSide(side));
  }

  /**
   * Creates a command that sets the coral station pickup position to a specific value.
   * Directly sets the position selection without cycling through options.
   *
   * @param position The position to set
   * @return Command that will set the position when executed
   */
  public Command runSetCoralStationPickupPosition(
          FieldConstants.CoralStationPickupPosition position) {
    return this.runOnce(() -> state.setCoralStationPickupPosition(position));
  }
}