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
package frc.alotobots.reefscape.subsystems.autocycle.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.FieldConstants;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * Manages state for the AutoCycle subsystem, including selection state and pathfinding control.
 * This class consolidates all state management including reef/level selections, coral station
 * selections, and pathfinding control state.
 */
public class AutoCycleState {
  /** Logging key prefixes */
  private static final String LOG_PREFIX = "AutoCycle";

  private static final String REEF_KEY = LOG_PREFIX + "/Reef";
  private static final String CORAL_STATION_KEY = LOG_PREFIX + "/CoralStation";
  private static final String PATHFINDING_KEY = LOG_PREFIX + "/Pathfinding";

  /**
   * Represents which type of pathfinding was last activated. Used to determine when to auto-rerun
   * pathfinding on selection changes.
   */
  public enum ActivePathfindingType {
    /** No pathfinding is active */
    NONE,

    /** Pathfinding to a reef target is active */
    REEF,

    /** Pathfinding to a coral station is active */
    CORAL_STATION
  }

  // Selection state
  /** The currently selected reef branch */
  @Getter @Setter private FieldConstants.ReefBranch reefBranch;

  /** The currently selected reef level */
  @Getter @Setter private FieldConstants.Level reefLevel;

  /** The currently selected coral station side */
  @Getter @Setter private FieldConstants.CoralStationSide coralStationSide;

  /** The currently selected coral station pickup position */
  @Getter @Setter private FieldConstants.CoralStationPickupPosition coralStationPickupPosition;

  // Pathfinding control state
  /** Currently running pathfinding command */
  @Getter private Command activePathfindingCommand;

  /** Whether pathfinding is currently enabled */
  @Getter private boolean pathfindingEnabled = false;

  /** Last type of pathfinding that was activated */
  @Getter private ActivePathfindingType lastActiveType = ActivePathfindingType.NONE;

  /** Whether pathfinding is currently paused */
  @Getter private boolean isPaused = false;

  /**
   * Creates a new AutoCycleState with default values.
   *
   * @return A new AutoCycleState instance with default selections
   */
  public static AutoCycleState createDefault() {
    AutoCycleState state = new AutoCycleState();
    state.reefBranch = FieldConstants.ReefBranch.A;
    state.reefLevel = FieldConstants.Level.L2;
    state.coralStationSide = FieldConstants.CoralStationSide.RIGHT;
    state.coralStationPickupPosition = FieldConstants.CoralStationPickupPosition.P1;
    return state;
  }

  /**
   * Sets whether pathfinding is enabled.
   *
   * @param enabled True to enable pathfinding, false to disable
   */
  public void setPathfindingEnabled(boolean enabled) {
    this.pathfindingEnabled = enabled;
    Logger.recordOutput(PATHFINDING_KEY + "/Enabled", enabled);
  }

  /**
   * Sets which type of pathfinding is currently active.
   *
   * @param type The type of pathfinding to activate
   */
  public void setActivePathfinding(ActivePathfindingType type) {
    this.lastActiveType = type;
    Logger.recordOutput(PATHFINDING_KEY + "/ActiveType", type.toString());
  }

  /**
   * Sets the active pathfinding command and logs the change.
   *
   * @param command The command to set as active pathfinding command
   */
  public void setActivePathfindingCommand(Command command) {
    this.activePathfindingCommand = command;
    Logger.recordOutput(
        PATHFINDING_KEY + "/ActiveCommandName", command != null ? command.getName() : "none");
  }

  /**
   * Sets whether pathfinding is currently paused.
   *
   * @param paused True to pause pathfinding, false to resume
   */
  public void setPaused(boolean paused) {
    this.isPaused = paused;
    Logger.recordOutput(PATHFINDING_KEY + "/IsPaused", paused);
  }

  /**
   * Gets the path name for the currently selected branch.
   *
   * @return Path name formatted as "BranchApproach_[branch]"
   */
  public String getSelectedReefBranchPathName() {
    return String.format("BranchApproach_%s", reefBranch);
  }

  /**
   * Gets the path name for the currently selected coral station position.
   *
   * @return Path name formatted as "CoralStationApproach_[side]_[position]"
   */
  public String getSelectedCoralStationPathName() {
    return String.format(
        "CoralStationApproach_%s_%s", coralStationSide, coralStationPickupPosition);
  }

  /**
   * Checks if a change in reef selections should trigger a pathfinding rerun.
   *
   * @return true if reef pathfinding was last active and not paused
   */
  public boolean shouldRerunReefPathfinding() {
    return lastActiveType == ActivePathfindingType.REEF && !isPaused;
  }

  /**
   * Checks if a change in coral station selections should trigger a pathfinding rerun.
   *
   * @return true if coral station pathfinding was last active and not paused
   */
  public boolean shouldRerunCoralStationPathfinding() {
    return lastActiveType == ActivePathfindingType.CORAL_STATION && !isPaused;
  }

  /**
   * Logs all state information to AdvantageKit for debugging and monitoring. This method should be
   * called periodically to update telemetry.
   */
  public void logState() {
    // Log selection state
    Logger.recordOutput(REEF_KEY + "/Branch", reefBranch.name());
    Logger.recordOutput(REEF_KEY + "/Level", reefLevel.name());
    Logger.recordOutput(CORAL_STATION_KEY + "/Side", coralStationSide.name());
    Logger.recordOutput(CORAL_STATION_KEY + "/Position", coralStationPickupPosition.name());

    // Log pathfinding state
    Logger.recordOutput(PATHFINDING_KEY + "/Enabled", pathfindingEnabled);
    Logger.recordOutput(PATHFINDING_KEY + "/ActiveType", lastActiveType.toString());
    Logger.recordOutput(PATHFINDING_KEY + "/IsPaused", isPaused);
    Logger.recordOutput(PATHFINDING_KEY + "/ReefPath", getSelectedReefBranchPathName());
    Logger.recordOutput(PATHFINDING_KEY + "/CoralStationPath", getSelectedCoralStationPathName());
  }

  /**
   * Logs target poses for debugging and visualization.
   *
   * @param reefBranchPose Optional end pose for reef branch path
   * @param coralStationPose Optional end pose for coral station path
   */
  public void logTargetPoses(Optional<Pose2d> reefBranchPose, Optional<Pose2d> coralStationPose) {
    reefBranchPose.ifPresent(pose -> Logger.recordOutput(REEF_KEY + "/TargetPose", pose));

    coralStationPose.ifPresent(
        pose -> Logger.recordOutput(CORAL_STATION_KEY + "/TargetPose", pose));
  }

  /**
   * Cycles the reef branch selection to the next branch to the right. Wraps around to the first
   * branch if at the end.
   */
  public void cycleReefBranchRight() {
    int nextOrdinal = reefBranch.ordinal() + 1;
    if (nextOrdinal >= FieldConstants.ReefBranch.values().length) {
      nextOrdinal = 0;
    }
    setReefBranch(FieldConstants.ReefBranch.values()[nextOrdinal]);
  }

  /**
   * Cycles the reef branch selection to the next branch to the left. Wraps around to the last
   * branch if at the beginning.
   */
  public void cycleReefBranchLeft() {
    int nextOrdinal = reefBranch.ordinal() - 1;
    if (nextOrdinal < 0) {
      nextOrdinal = FieldConstants.ReefBranch.values().length - 1;
    }
    setReefBranch(FieldConstants.ReefBranch.values()[nextOrdinal]);
  }

  /** Cycles the reef level selection up. Does not wrap; stops at the highest level. */
  public void cycleReefLevelUp() {
    int nextOrdinal = reefLevel.ordinal() + 1;
    if (nextOrdinal < FieldConstants.Level.values().length) {
      setReefLevel(FieldConstants.Level.values()[nextOrdinal]);
    }
  }

  /** Cycles the reef level selection down. Does not wrap; stops at the lowest level. */
  public void cycleReefLevelDown() {
    int nextOrdinal = reefLevel.ordinal() - 1;
    if (nextOrdinal >= 0) {
      setReefLevel(FieldConstants.Level.values()[nextOrdinal]);
    }
  }

  /**
   * Cycles the coral station side selection to the right. Does not wrap; stops at the rightmost
   * side.
   */
  public void cycleCoralStationSideRight() {
    int nextOrdinal = coralStationSide.ordinal() + 1;
    if (nextOrdinal < FieldConstants.CoralStationSide.values().length) {
      setCoralStationSide(FieldConstants.CoralStationSide.values()[nextOrdinal]);
    }
  }

  /**
   * Cycles the coral station side selection to the left. Does not wrap; stops at the leftmost side.
   */
  public void cycleCoralStationSideLeft() {
    int nextOrdinal = coralStationSide.ordinal() - 1;
    if (nextOrdinal >= 0) {
      setCoralStationSide(FieldConstants.CoralStationSide.values()[nextOrdinal]);
    }
  }

  /**
   * Cycles the coral station position selection to the right. Does not wrap; stops at the rightmost
   * position.
   */
  public void cycleCoralStationPositionRight() {
    int nextOrdinal = coralStationPickupPosition.ordinal() + 1;
    if (nextOrdinal < FieldConstants.CoralStationPickupPosition.values().length) {
      setCoralStationPickupPosition(
          FieldConstants.CoralStationPickupPosition.values()[nextOrdinal]);
    }
  }

  /**
   * Cycles the coral station position selection to the left. Does not wrap; stops at the leftmost
   * position.
   */
  public void cycleCoralStationPositionLeft() {
    int nextOrdinal = coralStationPickupPosition.ordinal() - 1;
    if (nextOrdinal >= 0) {
      setCoralStationPickupPosition(
          FieldConstants.CoralStationPickupPosition.values()[nextOrdinal]);
    }
  }
}
