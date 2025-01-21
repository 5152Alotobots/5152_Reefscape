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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.reefscape.FieldConstants;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToCoralStation;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToReef;
import frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState;
import lombok.Getter;

public class AutoCycleSubsystem extends SubsystemBase {

  @Getter private final AutoCycleState state;
  @Getter private final PathPlannerManager pathPlannerManager;
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  public AutoCycleSubsystem(
      PathPlannerManager pathPlannerManager, SwerveDriveSubsystem swerveDriveSubsystem) {
    this.state = AutoCycleState.createDefault();
    this.pathPlannerManager = pathPlannerManager;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
  }

  @Override
  public void periodic() {
    state.logState();
  }

  /** Command to toggle pathfinding enabled state */
  public Command togglePathfinding() {
    return runOnce(
            () -> {
              state.setPathfindingEnabled(!state.isPathfindingEnabled());
              if (!state.isPathfindingEnabled()) {
                cancelActivePathfinding();
              }
            })
        .ignoringDisable(true);
  }

  /** Cancels the active pathfinding command if one exists */
  public void cancelActivePathfinding() {
    Command currentCommand = state.getActivePathfindingCommand();
    if (currentCommand != null) {
      currentCommand.cancel();
      state.setActivePathfindingCommand(null);
    }
  }

  // Reef branch selection commands
  public Command cycleReefBranchRight(boolean replan) {
    return handleReefBranchChange(() -> state.cycleReefBranchRight(), replan);
  }

  public Command cycleReefBranchLeft(boolean replan) {
    return handleReefBranchChange(() -> state.cycleReefBranchLeft(), replan);
  }

  private Command handleReefBranchChange(Runnable changeAction, boolean replan) {
    return runOnce(
        () -> {
          if (replan) {
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
            new PathfindToReef(this).schedule();
          }
        });
  }

  // Reef level selection commands
  public Command cycleReefLevelUp(boolean replan) {
    return handleReefLevelChange(() -> state.cycleReefLevelUp(), replan);
  }

  public Command cycleReefLevelDown(boolean replan) {
    return handleReefLevelChange(() -> state.cycleReefLevelDown(), replan);
  }

  private Command handleReefLevelChange(Runnable changeAction, boolean replan) {
    return runOnce(
        () -> {
          if (replan) {
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
            new PathfindToReef(this).schedule();
          }
        });
  }

  // Coral station side selection commands
  public Command cycleCoralStationSideRight(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationSideRight(), replan);
  }

  public Command cycleCoralStationSideLeft(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationSideLeft(), replan);
  }

  // Coral station position selection commands
  public Command cycleCoralStationPositionRight(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationPositionRight(), replan);
  }

  public Command cycleCoralStationPositionLeft(boolean replan) {
    return handleCoralStationChange(() -> state.cycleCoralStationPositionLeft(), replan);
  }

  private Command handleCoralStationChange(Runnable changeAction, boolean replan) {
    return runOnce(
        () -> {
          if (replan) {
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
            new PathfindToCoralStation(this).schedule();
          }
        });
  }

  // Direct selection commands
  public Command setReefBranch(FieldConstants.ReefBranch branch, boolean replan) {
    return handleReefBranchChange(() -> state.setReefBranch(branch), replan);
  }

  public Command setReefLevel(FieldConstants.Level level, boolean replan) {
    return handleReefLevelChange(() -> state.setReefLevel(level), replan);
  }

  public Command setCoralStationSide(FieldConstants.CoralStationSide side, boolean replan) {
    return handleCoralStationChange(() -> state.setCoralStationSide(side), replan);
  }

  public Command setCoralStationPosition(
      FieldConstants.CoralStationPickupPosition position, boolean replan) {
    return handleCoralStationChange(() -> state.setCoralStationPickupPosition(position), replan);
  }

  // Convenience methods that default to replanning
  public Command cycleReefBranchRight() {
    return cycleReefBranchRight(true);
  }

  public Command cycleReefBranchLeft() {
    return cycleReefBranchLeft(true);
  }

  public Command cycleReefLevelUp() {
    return cycleReefLevelUp(true);
  }

  public Command cycleReefLevelDown() {
    return cycleReefLevelDown(true);
  }

  public Command cycleCoralStationSideRight() {
    return cycleCoralStationSideRight(true);
  }

  public Command cycleCoralStationSideLeft() {
    return cycleCoralStationSideLeft(true);
  }

  public Command cycleCoralStationPositionRight() {
    return cycleCoralStationPositionRight(true);
  }

  public Command cycleCoralStationPositionLeft() {
    return cycleCoralStationPositionLeft(true);
  }

  public Command setReefBranch(FieldConstants.ReefBranch branch) {
    return setReefBranch(branch, true);
  }

  public Command setReefLevel(FieldConstants.Level level) {
    return setReefLevel(level, true);
  }

  public Command setCoralStationSide(FieldConstants.CoralStationSide side) {
    return setCoralStationSide(side, true);
  }

  public Command setCoralStationPosition(FieldConstants.CoralStationPickupPosition position) {
    return setCoralStationPosition(position, true);
  }
}
