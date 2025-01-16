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
package frc.alotobots.reefscape.subsystems.autocycle.reef.util;

import frc.alotobots.reefscape.FieldConstants;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

/**
 * Encapsulates the state of the AutoCycle system, including reef branch, level, and coral station
 * selections.
 */
@Builder
public class AutoCycleState {
  /** The currently selected reef branch for navigation. */
  @Getter @Setter private FieldConstants.ReefBranch reefBranch;

  /** The currently selected level for reef interaction. */
  @Getter @Setter private FieldConstants.Level reefLevel;

  /** The currently selected side for human player pickup */
  @Getter @Setter private FieldConstants.CoralStationSide coralStationSide;

  /** The currently selected position in human player pickup */
  @Getter @Setter private FieldConstants.CoralStationPickupPosition coralStationPickupPosition;

  /** Flag indicating if currently pathfinding to coral station */
  @Getter @Setter private boolean isPathfindingToCoralStation;

  /** Flag indicating if currently pathfinding to reef branch */
  @Getter @Setter private boolean isPathfindingToReefBranch;

  /** The name of the currently active path being followed */
  @Getter @Setter private String activePath;

  /**
   * Creates a new AutoCycleState with default values.
   *
   * @return A new AutoCycleState instance initialized with default values
   */
  public static AutoCycleState createDefault() {
    return AutoCycleState.builder()
        .reefBranch(FieldConstants.ReefBranch.A)
        .reefLevel(FieldConstants.Level.L2)
        .coralStationSide(FieldConstants.CoralStationSide.RIGHT)
        .coralStationPickupPosition(FieldConstants.CoralStationPickupPosition.P1)
        .isPathfindingToCoralStation(false)
        .isPathfindingToReefBranch(false)
        .activePath("")
        .build();
  }

  /**
   * Gets the path name for the currently selected branch and level combination.
   *
   * @return Current path name based on selected branch and level
   */
  public String getSelectedReefBranchPathName() {
    return String.format("BranchApproach_%s_%s", reefBranch, reefLevel);
  }

  /**
   * Gets the path name for the currently selected coral station side and position combination.
   *
   * @return Current path name based on selected side and position
   */
  public String getSelectedCoralStationPathName() {
    return String.format(
        "CoralStationApproach_%s_%s", coralStationSide, coralStationPickupPosition);
  }

  /** Logs the current output state to AdvantageKit. */
  public void logState() {
    Logger.recordOutput("AutoCycleReef/Reef/Branch", reefBranch.name());
    Logger.recordOutput("AutoCycleReef/Reef/Level", reefLevel.name());
    Logger.recordOutput("AutoCycleReef/CoralStation/Side", coralStationSide.name());
    Logger.recordOutput("AutoCycleReef/CoralStation/Position", coralStationPickupPosition.name());
    Logger.recordOutput("AutoCycleReef/Pathfinding/ToCoralStation", isPathfindingToCoralStation);
    Logger.recordOutput("AutoCycleReef/Pathfinding/ToReefBranch", isPathfindingToReefBranch);
    Logger.recordOutput("AutoCycleReef/Pathfinding/ActivePath", activePath);

    // Log the current target paths
    Logger.recordOutput(
        "AutoCycleReef/Pathfinding/ReefBranchPath", getSelectedReefBranchPathName());
    Logger.recordOutput(
        "AutoCycleReef/Pathfinding/CoralStationPath", getSelectedCoralStationPathName());
  }
}
