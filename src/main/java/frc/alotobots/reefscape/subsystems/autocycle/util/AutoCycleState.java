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
    NONE,
    REEF,
    CORAL_STATION
  }

  // Selection state
  @Getter @Setter private FieldConstants.ReefBranch reefBranch;
  @Getter @Setter private FieldConstants.Level reefLevel;
  @Getter @Setter private FieldConstants.CoralStationSide coralStationSide;
  @Getter @Setter private FieldConstants.CoralStationPickupPosition coralStationPickupPosition;

  // Pathfinding control state
  /** Currently running pathfinding command */
  @Getter private Command activePathfindingCommand;

  @Getter private boolean pathfindingEnabled = false;
  @Getter private ActivePathfindingType lastActiveType = ActivePathfindingType.NONE;
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

  /** Sets the active pathfinding command and logs the change */
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
   * Gets the path name for the currently selected branch and level.
   *
   * @return Path name formatted as "BranchApproach_[branch]_[level]"
   */
  public String getSelectedReefBranchPathName() {
    return String.format("BranchApproach_%s_%s", reefBranch, reefLevel);
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
   * @return true if reef pathfinding was last active
   */
  public boolean shouldRerunReefPathfinding() {
    return lastActiveType == ActivePathfindingType.REEF && !isPaused;
  }

  /**
   * Checks if a change in coral station selections should trigger a pathfinding rerun.
   *
   * @return true if coral station pathfinding was last active
   */
  public boolean shouldRerunCoralStationPathfinding() {
    return lastActiveType == ActivePathfindingType.CORAL_STATION && !isPaused;
  }

  /** Logs all state information to AdvantageKit for debugging and monitoring. */
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
}
