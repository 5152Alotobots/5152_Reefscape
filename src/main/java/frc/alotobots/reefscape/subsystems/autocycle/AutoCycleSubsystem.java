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
import frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState;
import frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState.ActivePathfindingType;
import java.util.Optional;
import lombok.Getter;

/**
 * Subsystem that manages automated movement to reef and coral station positions. Provides
 * high-level pathfinding commands and selection management for driver assist.
 */
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

  /** Cancels the active pathfinding command if one exists */
  public void cancelActivePathfinding() {
    Command currentCommand = state.getActivePathfindingCommand();
    if (currentCommand != null) {
      currentCommand.cancel();
      state.setActivePathfindingCommand(null);
    }
  }

  /**
   * Creates command to pathfind to currently selected reef branch. Will switch to precision
   * alignment when near target.
   */
  public Command pathfindToReef() {
    return Commands.either(
        // When near target, switch to precision alignment
        this.defer(
            () -> {
              Optional<Pose2d> targetPose =
                  pathPlannerManager.getPathEndPose(state.getSelectedReefBranchPathName());
              Command alignCommand =
                  targetPose
                      .map(pose -> new DrivePrecisionAlign(swerveDriveSubsystem).getCommand(pose))
                      .orElse(new PrintCommand("No valid reef target pose!"));
              state.setActivePathfindingCommand(alignCommand);
              return alignCommand;
            }),
        // Otherwise start/continue pathfinding
        Commands.runOnce(
            () -> {
              state.setActivePathfinding(AutoCycleState.ActivePathfindingType.REEF);
              Command pathCommand =
                  pathPlannerManager.getPathfindThenFollowPathCommand(
                      state.getSelectedReefBranchPathName());
              state.setActivePathfindingCommand(pathCommand);
              pathCommand.schedule();
            }),
        // Condition for switching to precision
        () ->
            pathPlannerManager.nearEndOfPath(
                state.getSelectedReefBranchPathName(),
                ALIGNMENT_TRANSLATION_TOLERANCE,
                ALIGNMENT_ROTATION_TOLERANCE));
  }

  /**
   * Creates command to pathfind to currently selected coral station. Will switch to precision
   * alignment when near target.
   */
  public Command pathfindToCoralStation() {
    return Commands.either(
        // When near target, switch to precision alignment
        this.defer(
            () -> {
              Optional<Pose2d> targetPose =
                  pathPlannerManager.getPathEndPose(state.getSelectedCoralStationPathName());
              Command alignCommand =
                  targetPose
                      .map(pose -> new DrivePrecisionAlign(swerveDriveSubsystem).getCommand(pose))
                      .orElse(new PrintCommand("No valid coral station target pose!"));
              state.setActivePathfindingCommand(alignCommand);
              return alignCommand;
            }),
        // Otherwise start/continue pathfinding
        Commands.runOnce(
            () -> {
              state.setActivePathfinding(AutoCycleState.ActivePathfindingType.CORAL_STATION);
              Command pathCommand =
                  pathPlannerManager.getPathfindThenFollowPathCommand(
                      state.getSelectedCoralStationPathName());
              state.setActivePathfindingCommand(pathCommand);
              pathCommand.schedule();
            }),
        // Condition for switching to precision
        () ->
            pathPlannerManager.nearEndOfPath(
                state.getSelectedCoralStationPathName(),
                ALIGNMENT_TRANSLATION_TOLERANCE,
                ALIGNMENT_ROTATION_TOLERANCE));
  }

  /** Creates command to stop any active pathfinding. */
  public Command stopPathfinding() {
    return runOnce(
        () -> {
          state.setActivePathfinding(ActivePathfindingType.NONE);
          state.setPaused(false);
        });
  }

  // Reef branch selection commands
  public Command cycleReefBranchRight() {
    return handleReefBranchChange(
        () -> {
          int nextOrdinal = state.getReefBranch().ordinal() + 1;
          if (nextOrdinal >= FieldConstants.ReefBranch.values().length) {
            nextOrdinal = 0;
          }
          state.setReefBranch(FieldConstants.ReefBranch.values()[nextOrdinal]);
        });
  }

  public Command cycleReefBranchLeft() {
    return handleReefBranchChange(
        () -> {
          int nextOrdinal = state.getReefBranch().ordinal() - 1;
          if (nextOrdinal < 0) {
            nextOrdinal = FieldConstants.ReefBranch.values().length - 1;
          }
          state.setReefBranch(FieldConstants.ReefBranch.values()[nextOrdinal]);
        });
  }

  private Command handleReefBranchChange(Runnable changeAction) {
    return runOnce(
        () -> {
          changeAction.run();
          if (state.shouldRerunReefPathfinding()) {
            pathfindToReef().schedule();
          }
        });
  }

  // Reef level selection commands
  public Command cycleReefLevelUp() {
    return handleReefLevelChange(
        () -> {
          int nextOrdinal = state.getReefLevel().ordinal() + 1;
          if (nextOrdinal < FieldConstants.Level.values().length) {
            state.setReefLevel(FieldConstants.Level.values()[nextOrdinal]);
          }
        });
  }

  public Command cycleReefLevelDown() {
    return handleReefLevelChange(
        () -> {
          int nextOrdinal = state.getReefLevel().ordinal() - 1;
          if (nextOrdinal >= 0) {
            state.setReefLevel(FieldConstants.Level.values()[nextOrdinal]);
          }
        });
  }

  private Command handleReefLevelChange(Runnable changeAction) {
    return runOnce(
        () -> {
          changeAction.run();
          if (state.shouldRerunReefPathfinding()) {
            pathfindToReef().schedule();
          }
        });
  }

  // Coral station side selection commands
  public Command cycleCoralStationSideRight() {
    return handleCoralStationChange(
        () -> {
          int nextOrdinal = state.getCoralStationSide().ordinal() + 1;
          if (nextOrdinal < FieldConstants.CoralStationSide.values().length) {
            state.setCoralStationSide(FieldConstants.CoralStationSide.values()[nextOrdinal]);
          }
        });
  }

  public Command cycleCoralStationSideLeft() {
    return handleCoralStationChange(
        () -> {
          int nextOrdinal = state.getCoralStationSide().ordinal() - 1;
          if (nextOrdinal >= 0) {
            state.setCoralStationSide(FieldConstants.CoralStationSide.values()[nextOrdinal]);
          }
        });
  }

  // Coral station position selection commands
  public Command cycleCoralStationPositionRight() {
    return handleCoralStationChange(
        () -> {
          int nextOrdinal = state.getCoralStationPickupPosition().ordinal() + 1;
          if (nextOrdinal < FieldConstants.CoralStationPickupPosition.values().length) {
            state.setCoralStationPickupPosition(
                FieldConstants.CoralStationPickupPosition.values()[nextOrdinal]);
          }
        });
  }

  public Command cycleCoralStationPositionLeft() {
    return handleCoralStationChange(
        () -> {
          int nextOrdinal = state.getCoralStationPickupPosition().ordinal() - 1;
          if (nextOrdinal >= 0) {
            state.setCoralStationPickupPosition(
                FieldConstants.CoralStationPickupPosition.values()[nextOrdinal]);
          }
        });
  }

  private Command handleCoralStationChange(Runnable changeAction) {
    return runOnce(
        () -> {
          changeAction.run();
          if (state.shouldRerunCoralStationPathfinding()) {
            pathfindToCoralStation().schedule();
          }
        });
  }

  // Direct selection commands
  public Command setReefBranch(FieldConstants.ReefBranch branch) {
    return handleReefBranchChange(() -> state.setReefBranch(branch));
  }

  public Command setReefLevel(FieldConstants.Level level) {
    return handleReefLevelChange(() -> state.setReefLevel(level));
  }

  public Command setCoralStationSide(FieldConstants.CoralStationSide side) {
    return handleCoralStationChange(() -> state.setCoralStationSide(side));
  }

  public Command setCoralStationPosition(FieldConstants.CoralStationPickupPosition position) {
    return handleCoralStationChange(() -> state.setCoralStationPickupPosition(position));
  }
}
