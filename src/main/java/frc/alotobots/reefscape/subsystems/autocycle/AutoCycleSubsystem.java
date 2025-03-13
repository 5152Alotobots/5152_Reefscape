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

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.reefscape.FieldConstants;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToCoralStation;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToReef;
import frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Subsystem that manages automated navigation to reef and coral station targets. Provides methods
 * for selecting target locations on the field and pathfinding to those locations using
 * PathPlanner's pathfinding capabilities.
 *
 * <p>This subsystem allows drivers to:
 *
 * <ul>
 *   <li>Select reef branches and levels to navigate to
 *   <li>Select coral station sides and positions to navigate to
 *   <li>Toggle pathfinding functionality on and off
 *   <li>Manually override pathfinding with driver control
 * </ul>
 */
public class AutoCycleSubsystem extends SubsystemBase {

  @Getter private final AutoCycleState state;
  @Getter private final PathPlannerManager pathPlannerManager;
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final Supplier<ChassisSpeeds> manualControlChassisSpeeds;
  private final Field2d field = new Field2d();

  /**
   * Creates a new AutoCycleSubsystem.
   *
   * @param pathPlannerManager The PathPlannerManager used for path planning and following
   * @param swerveDriveSubsystem The swerve drive subsystem to control for movement
   * @param manualControlChassisSpeeds A supplier for manual driver input as chassis speeds
   */
  public AutoCycleSubsystem(
      PathPlannerManager pathPlannerManager,
      SwerveDriveSubsystem swerveDriveSubsystem,
      Supplier<ChassisSpeeds> manualControlChassisSpeeds) {
    this.state = AutoCycleState.createDefault();
    this.pathPlannerManager = pathPlannerManager;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.manualControlChassisSpeeds = manualControlChassisSpeeds;
  }

  /**
   * Periodic method that runs on each scheduler loop. Updates and logs state information and target
   * poses.
   */
  @Override
  public void periodic() {
    // Log robot to field widget for drivers
    field.setRobotPose(swerveDriveSubsystem.getPose());

    state.logState();
    // Get end poses for currently selected paths
    Optional<Pose2d> reefPose =
        pathPlannerManager.getPathEndPose(state.getSelectedReefBranchPathName());

    Optional<Pose2d> coralStationPose =
        pathPlannerManager.getPathEndPose(state.getSelectedCoralStationPathName());

    // Log the poses using existing method
    state.logTargetPoses(reefPose, coralStationPose);

    // Update the field widget with the target poses
      reefPose.ifPresent(pose2d -> field.getObject("ReefTarget").setPoses(pose2d));
      coralStationPose.ifPresent(pose2d -> field.getObject("CoralStationTarget").setPoses(pose2d));
  }

  /**
   * Creates a command to toggle pathfinding enabled state.
   *
   * @return Command that toggles pathfinding on/off when executed
   */
  public Command togglePathfinding() {
    return Commands.runOnce(
            () -> {
              state.setPathfindingEnabled(!state.isPathfindingEnabled());
              if (!state.isPathfindingEnabled()) {
                cancelActivePathfinding();
              }
            })
        .ignoringDisable(true);
  }

  /**
   * Cancels the active pathfinding command if one exists. This stops the robot from following the
   * current path.
   */
  public void cancelActivePathfinding() {
    Command currentCommand = state.getActivePathfindingCommand();
    if (currentCommand != null) {
      currentCommand.cancel();
      state.setActivePathfindingCommand(null);
    }
  }

  /**
   * Creates a command to cycle reef branch selection to the right.
   *
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that cycles reef branch right when executed
   */
  public Command cycleReefBranchRight(boolean replan) {
    return handleReefBranchChange(() -> state.cycleReefBranchRight(), replan);
  }

  /**
   * Creates a command to cycle reef branch selection to the left.
   *
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that cycles reef branch left when executed
   */
  public Command cycleReefBranchLeft(boolean replan) {
    return handleReefBranchChange(() -> state.cycleReefBranchLeft(), replan);
  }

  /**
   * Helper method to handle reef branch changes. Manages state changes and replanning when reef
   * branch selection changes.
   *
   * @param changeAction The runnable action that changes the reef branch state
   * @param replan Whether to automatically replan pathfinding
   * @return Command that executes the change action
   */
  private Command handleReefBranchChange(Runnable changeAction, boolean replan) {
    return Commands.runOnce(
        () -> {
          if (replan && state.getLastActiveType() == AutoCycleState.ActivePathfindingType.REEF) {
            // Cancel any existing pathfinding command
            Command currentCommand = state.getActivePathfindingCommand();
            if (currentCommand != null) {
              currentCommand.cancel();
              state.setActivePathfindingCommand(null);
            }
          }

          // Run the state change action
          changeAction.run();

          // Schedule new pathfinding if needed
          if (replan && state.shouldRerunReefPathfinding()) {
            new PathfindToReef(this, manualControlChassisSpeeds).schedule();
          }
        });
  }

  /**
   * Creates a command to cycle reef level selection up.
   *
   * @param replan Whether to automatically replan (not used but kept for API consistency)
   * @return Command that cycles reef level up when executed
   */
  public Command cycleReefLevelUp(boolean replan) {
    return handleReefLevelChange(() -> state.cycleReefLevelUp());
  }

  /**
   * Creates a command to cycle reef level selection down.
   *
   * @param replan Whether to automatically replan (not used but kept for API consistency)
   * @return Command that cycles reef level down when executed
   */
  public Command cycleReefLevelDown(boolean replan) {
    return handleReefLevelChange(() -> state.cycleReefLevelDown());
  }

  /**
   * Helper method to handle reef level changes.
   *
   * @param changeAction The runnable action that changes the reef level state
   * @return Command that executes the change action
   */
  private Command handleReefLevelChange(Runnable changeAction) {
    // Run the state change action
    return runOnce(changeAction::run);
  }

  /**
   * Creates a command to cycle coral station side selection to the right.
   *
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that cycles coral station side right when executed
   */
  public Command cycleCoralStationSideRight(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationSideRight(), replan);
  }

  /**
   * Creates a command to cycle coral station side selection to the left.
   *
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that cycles coral station side left when executed
   */
  public Command cycleCoralStationSideLeft(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationSideLeft(), replan);
  }

  /**
   * Creates a command to cycle coral station position selection to the right.
   *
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that cycles coral station position right when executed
   */
  public Command cycleCoralStationPositionRight(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationPositionRight(), replan);
  }

  /**
   * Creates a command to cycle coral station position selection to the left.
   *
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that cycles coral station position left when executed
   */
  public Command cycleCoralStationPositionLeft(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationPositionLeft(), replan);
  }

  /**
   * Helper method to handle coral station changes. Manages state changes and replanning when coral
   * station selection changes.
   *
   * @param changeAction The runnable action that changes the coral station state
   * @param replan Whether to automatically replan pathfinding
   * @return Command that executes the change action
   */
  private Command handleCoralStationChange(Runnable changeAction, boolean replan) {
    return Commands.runOnce(
        () -> {
          if (replan
              && state.getLastActiveType() == AutoCycleState.ActivePathfindingType.CORAL_STATION) {
            // Cancel any existing pathfinding command
            Command currentCommand = state.getActivePathfindingCommand();
            if (currentCommand != null) {
              currentCommand.cancel();
              state.setActivePathfindingCommand(null);
            }
          }

          // Run the state change action
          changeAction.run();

          // Schedule new pathfinding if needed
          if (replan && state.shouldRerunCoralStationPathfinding()) {
            new PathfindToCoralStation(this, manualControlChassisSpeeds).schedule();
          }
        });
  }

  /**
   * Creates a command to set the reef branch directly.
   *
   * @param branch The reef branch to select
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that sets the reef branch when executed
   */
  public Command setReefBranch(FieldConstants.ReefBranch branch, boolean replan) {
    return handleReefBranchChange(() -> state.setReefBranch(branch), replan);
  }

  /**
   * Creates a command to set the reef level directly.
   *
   * @param level The reef level to select
   * @return Command that sets the reef level when executed
   */
  public Command setReefLevel(FieldConstants.Level level) {
    return handleReefLevelChange(() -> state.setReefLevel(level));
  }

  /**
   * Creates a command to set the coral station side directly.
   *
   * @param side The coral station side to select
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that sets the coral station side when executed
   */
  public Command setCoralStationSide(FieldConstants.CoralStationSide side, boolean replan) {
    return handleCoralStationChange(() -> state.setCoralStationSide(side), replan);
  }

  /**
   * Creates a command to set the coral station position directly.
   *
   * @param position The coral station position to select
   * @param replan Whether to automatically replan pathfinding after selection change
   * @return Command that sets the coral station position when executed
   */
  public Command setCoralStationPosition(
      FieldConstants.CoralStationPickupPosition position, boolean replan) {
    return handleCoralStationChange(() -> state.setCoralStationPickupPosition(position), replan);
  }

  /**
   * Convenience method that defaults to replanning when cycling reef branch right.
   *
   * @return Command that cycles reef branch right and replans pathfinding
   */
  public Command cycleReefBranchRight() {
    return cycleReefBranchRight(true);
  }

  /**
   * Convenience method that defaults to replanning when cycling reef branch left.
   *
   * @return Command that cycles reef branch left and replans pathfinding
   */
  public Command cycleReefBranchLeft() {
    return cycleReefBranchLeft(true);
  }

  /**
   * Convenience method that defaults to replanning when cycling reef level up.
   *
   * @return Command that cycles reef level up and replans pathfinding
   */
  public Command cycleReefLevelUp() {
    return cycleReefLevelUp(true);
  }

  /**
   * Convenience method that defaults to replanning when cycling reef level down.
   *
   * @return Command that cycles reef level down and replans pathfinding
   */
  public Command cycleReefLevelDown() {
    return cycleReefLevelDown(true);
  }

  /**
   * Convenience method that defaults to replanning when cycling coral station side right.
   *
   * @return Command that cycles coral station side right and replans pathfinding
   */
  public Command cycleCoralStationSideRight() {
    return cycleCoralStationSideRight(true);
  }

  /**
   * Convenience method that defaults to replanning when cycling coral station side left.
   *
   * @return Command that cycles coral station side left and replans pathfinding
   */
  public Command cycleCoralStationSideLeft() {
    return cycleCoralStationSideLeft(true);
  }

  /**
   * Convenience method that defaults to replanning when cycling coral station position right.
   *
   * @return Command that cycles coral station position right and replans pathfinding
   */
  public Command cycleCoralStationPositionRight() {
    return cycleCoralStationPositionRight(true);
  }

  /**
   * Convenience method that defaults to replanning when cycling coral station position left.
   *
   * @return Command that cycles coral station position left and replans pathfinding
   */
  public Command cycleCoralStationPositionLeft() {
    return cycleCoralStationPositionLeft(true);
  }

  /**
   * Convenience method that defaults to replanning when setting reef branch.
   *
   * @param branch The reef branch to select
   * @return Command that sets the reef branch and replans pathfinding
   */
  public Command setReefBranch(FieldConstants.ReefBranch branch) {
    return setReefBranch(branch, true);
  }

  /**
   * Convenience method that defaults to replanning when setting coral station side.
   *
   * @param side The coral station side to select
   * @return Command that sets the coral station side and replans pathfinding
   */
  public Command setCoralStationSide(FieldConstants.CoralStationSide side) {
    return setCoralStationSide(side, true);
  }

  /**
   * Convenience method that defaults to replanning when setting coral station position.
   *
   * @param position The coral station position to select
   * @return Command that sets the coral station position and replans pathfinding
   */
  public Command setCoralStationPosition(FieldConstants.CoralStationPickupPosition position) {
    return setCoralStationPosition(position, true);
  }
}
