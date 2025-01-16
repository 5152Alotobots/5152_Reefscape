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
package frc.alotobots.reefscape.subsystems.autocycle.reef;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.reefscape.FieldConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that manages the automatic cycling of reef branches and levels for autonomous
 * navigation. This subsystem allows for selection and cycling through different branches and levels
 * of the reef, facilitating automated movement patterns during matches.
 */
public class AutoCycleReefSubsystem extends SubsystemBase {
  /** The currently selected reef branch for navigation. */
  @Getter private FieldConstants.ReefBranch selectedReefBranch;

  /** The currently selected level for reef interaction. */
  @Getter private FieldConstants.Level selectedReefLevel;

  /** The currently selected side for human player pickup */
  @Getter private FieldConstants.CoralStationSide selectedCoralStationSide;

  /** The currently selected position in human player pickup */
  @Getter private FieldConstants.CoralStationPickupPosition selectedCoralStationPickupPosition;

  /** The pathplanner manager instance to use for pathfinding */
  private PathPlannerManager pathPlannerManager;

  /**
   * Creates a new AutoCycleReefSubsystem with default branch and level selections. Initializes with
   * Branch A, Level 2, Right HP station, and Position 1 as default values
   */
  public AutoCycleReefSubsystem(PathPlannerManager pathPlannerManager) {
    selectedReefBranch = FieldConstants.ReefBranch.A; // Default to first branch
    selectedReefLevel = FieldConstants.Level.L2; // Default to the lowest level
    selectedCoralStationSide = FieldConstants.CoralStationSide.RIGHT;
    selectedCoralStationPickupPosition = FieldConstants.CoralStationPickupPosition.P1;

    this.pathPlannerManager = pathPlannerManager;
  }

  /**
   * Periodic function that runs every robot loop. Logs the current branch, level, and path
   * selections to the dashboard.
   */
  @Override
  public void periodic() {
    Logger.recordOutput("AutoCycleReef/Branch/Branch", selectedReefBranch.name());
    Logger.recordOutput("AutoCycleReef/Branch/Level", selectedReefLevel.name());
    Logger.recordOutput("AutoCycleReef/Branch/PathName", getSelectedReefBranchPathName());
    Logger.recordOutput("AutoCycleReef/CoralStation/Side", selectedCoralStationSide.name());
    Logger.recordOutput(
        "AutoCycleReef/CoralStation/Position", selectedCoralStationPickupPosition.name());
    Logger.recordOutput("AutoCycleReef/CoralStation/PathName", getSelectedCoralStationPathName());
  }

  /**
   * Cycles to the next reef branch to the right.
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
    int nextOrdinal = selectedReefBranch.ordinal() + 1;
    if (nextOrdinal >= FieldConstants.ReefBranch.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    selectedReefBranch = FieldConstants.ReefBranch.values()[nextOrdinal];
    return true;
  }

  /**
   * Cycles to the next reef branch to the left.
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
    int nextOrdinal = selectedReefBranch.ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.ReefBranch.values().length - 1;
    }
    selectedReefBranch = FieldConstants.ReefBranch.values()[nextOrdinal];
    return true;
  }

  /**
   * Cycles to the next level up.
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
    int nextOrdinal = selectedReefLevel.ordinal() + 1;
    if (nextOrdinal >= FieldConstants.Level.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    selectedReefLevel = FieldConstants.Level.values()[nextOrdinal];
    return true;
  }

  /**
   * Cycles to the next level down.
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
    int nextOrdinal = selectedReefLevel.ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.Level.values().length - 1;
    }
    selectedReefLevel = FieldConstants.Level.values()[nextOrdinal];
    return true;
  }

  /**
   * Cycles to the next coral station side to the right.
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
    int nextOrdinal = selectedCoralStationSide.ordinal() + 1;
    if (nextOrdinal >= FieldConstants.CoralStationSide.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    selectedCoralStationSide = FieldConstants.CoralStationSide.values()[nextOrdinal];
    return true;
  }

  /**
   * Cycles to the next coral station side to the left.
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
    int nextOrdinal = selectedCoralStationSide.ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.CoralStationSide.values().length - 1;
    }
    selectedCoralStationSide = FieldConstants.CoralStationSide.values()[nextOrdinal];
    return true;
  }

  /**
   * Cycles to the next pickup position to the right.
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
    int nextOrdinal = selectedCoralStationPickupPosition.ordinal() + 1;
    if (nextOrdinal >= FieldConstants.CoralStationPickupPosition.values().length) {
      if (!allowWrap) return false;
      nextOrdinal = 0;
    }
    selectedCoralStationPickupPosition =
        FieldConstants.CoralStationPickupPosition.values()[nextOrdinal];
    return true;
  }

  /**
   * Cycles to the next pickup position to the left.
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
    int nextOrdinal = selectedCoralStationPickupPosition.ordinal() - 1;
    if (nextOrdinal < 0) {
      if (!allowWrap) return false;
      nextOrdinal = FieldConstants.CoralStationPickupPosition.values().length - 1;
    }
    selectedCoralStationPickupPosition =
        FieldConstants.CoralStationPickupPosition.values()[nextOrdinal];
    return true;
  }

  /**
   * Gets the path name for approaching a specific reef branch at a given level.
   *
   * @param branch Target branch to approach
   * @param level Target level to approach at
   * @return Formatted string representing the path name
   */
  private String getReefBranchPathName(
      FieldConstants.ReefBranch branch, FieldConstants.Level level) {
    return String.format("BranchApproach_%s_%s", branch, level);
  }

  /**
   * Gets the path name for approaching a specific coral station at a given pickup position.
   *
   * @param side Target side of the coral station to approach
   * @param pickupPosition Target pickup position in the human player station (1-3, Left to Right)
   * @return Formatted string representing the path name
   */
  private String getCoralStationPathName(
      FieldConstants.CoralStationSide side,
      FieldConstants.CoralStationPickupPosition pickupPosition) {
    return String.format("CoralStationApproach_%s_%s", side, pickupPosition);
  }

  /**
   * Gets the path name for the currently selected branch and level combination.
   *
   * @return Current path name based on selected branch and level
   */
  public String getSelectedReefBranchPathName() {
    return getReefBranchPathName(selectedReefBranch, selectedReefLevel);
  }

  /**
   * Gets the path name for the currently selected coral station side and position combination.
   *
   * @return Current path name based on selected side and position
   */
  public String getSelectedCoralStationPathName() {
    return getCoralStationPathName(selectedCoralStationSide, selectedCoralStationPickupPosition);
  }

  /**
   * Gets the pathfinder for the currently selected branch and level combination.
   *
   * @return Pathfinding command for selected branch and level
   */
  public Command pathfindToSelectedReefBranchPathName() {
    return new InstantCommand(
        () ->
            pathPlannerManager
                .getPathfindThenFollowPathCommand(
                    getReefBranchPathName(selectedReefBranch, selectedReefLevel))
                .schedule());
  }

  /**
   * Gets the pathfinder for the currently selected coral station side and position combination.
   *
   * @return Pathfinding command for selected side and position
   */
  public Command pathfindToSelectedCoralStationPathName() {
    return new InstantCommand(
        () ->
            pathPlannerManager
                .getPathfindThenFollowPathCommand(
                    getCoralStationPathName(
                        selectedCoralStationSide, selectedCoralStationPickupPosition))
                .schedule());
  }

  /**
   * Creates a command that changes the selected reef branch to the one that is one position to the
   * right.
   *
   * @param allowWrap If true, wraps around to first branch when at last branch
   * @return Command that will change the selected branch when executed
   */
  public Command runCycleReefBranchRight(boolean allowWrap) {
    return this.runOnce(() -> cycleReefBranchRight(allowWrap));
  }

  /**
   * Creates a command that changes the selected reef branch to the one that is one position to the
   * left.
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
   * Creates a command that changes the selected coral station side to the one that is one position
   * to the right.
   *
   * @param allowWrap If true, wraps around from RIGHT to LEFT
   * @return Command that will change the selected side when executed
   */
  public Command runCycleCoralStationSideRight(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationSideRight(allowWrap));
  }

  /**
   * Creates a command that changes the selected coral station side to the one that is one position
   * to the left.
   *
   * @param allowWrap If true, wraps around from LEFT to RIGHT
   * @return Command that will change the selected side when executed
   */
  public Command runCycleCoralStationSideLeft(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationSideLeft(allowWrap));
  }

  /**
   * Creates a command that changes the selected pickup position to the one that is one position to
   * the right.
   *
   * @param allowWrap If true, wraps around from P3 back to P1
   * @return Command that will change the selected position when executed
   */
  public Command runCycleCoralStationPickupPositionRight(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationPickupPositionRight(allowWrap));
  }

  /**
   * Creates a command that changes the selected pickup position to the one that is one position to
   * the left.
   *
   * @param allowWrap If true, wraps around from P1 back to P3
   * @return Command that will change the selected position when executed
   */
  public Command runCycleCoralStationPickupPositionLeft(boolean allowWrap) {
    return this.runOnce(() -> cycleCoralStationPickupPositionLeft(allowWrap));
  }
}
