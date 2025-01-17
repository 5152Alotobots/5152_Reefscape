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
import frc.alotobots.reefscape.FieldConstants;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.Optional;

/**
 * Encapsulates the state of the AutoCycle system, including reef branch, level, and coral station
 * selections. This class manages the current state of automated navigation and positioning
 * within the game field, as well as centralized logging of all state information.
 */
@Builder
public class AutoCycleState {
  /** Prefix for all AutoCycleReef logging keys */
  private static final String LOG_PREFIX = "AutoCycleReef";

  /** Root key for reef-related logging */
  private static final String REEF_KEY = LOG_PREFIX + "/Reef";

  /** Root key for coral station-related logging */
  private static final String CORAL_STATION_KEY = LOG_PREFIX + "/CoralStation";

  /** Root key for pathfinding-related logging */
  private static final String PATHFINDING_KEY = LOG_PREFIX + "/Pathfinding";

  /**
   * The currently selected reef branch for navigation.
   * Determines which branch of the reef structure the robot will target.
   */
  @Getter @Setter private FieldConstants.ReefBranch reefBranch;

  /**
   * The currently selected level for reef interaction.
   * Determines the vertical height level the robot will target on the reef.
   */
  @Getter @Setter private FieldConstants.Level reefLevel;

  /**
   * The currently selected side for human player pickup.
   * Determines which side of the field the robot will approach for game piece collection.
   */
  @Getter @Setter private FieldConstants.CoralStationSide coralStationSide;

  /**
   * The currently selected position in human player pickup.
   * Specifies the exact position along the selected side for game piece collection.
   */
  @Getter @Setter private FieldConstants.CoralStationPickupPosition coralStationPickupPosition;

  /**
   * Flag indicating if currently pathfinding to coral station.
   * True when the robot is actively navigating to a coral station position.
   */
  @Getter @Setter private boolean isPathfindingToCoralStation;

  /**
   * Flag indicating if currently pathfinding to reef branch.
   * True when the robot is actively navigating to a reef branch position.
   */
  @Getter @Setter private boolean isPathfindingToReefBranch;

  /**
   * The name of the currently active path being followed.
   * Stores the identifier of the path currently being executed by the pathfinding system.
   */
  @Getter @Setter private String activePath;

  /**
   * Creates a new AutoCycleState with default values.
   * Initializes the state with predefined default selections for reef branch,
   * level, coral station side, and pickup position.
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
   * Constructs a path identifier string based on the current reef branch and level selections.
   *
   * @return Current path name formatted as "BranchApproach_[branch]_[level]"
   */
  public String getSelectedReefBranchPathName() {
    return String.format("BranchApproach_%s_%s", reefBranch, reefLevel);
  }

  /**
   * Gets the path name for the currently selected coral station side and position combination.
   * Constructs a path identifier string based on the current coral station selections.
   *
   * @return Current path name formatted as "CoralStationApproach_[side]_[position]"
   */
  public String getSelectedCoralStationPathName() {
    return String.format(
            "CoralStationApproach_%s_%s", coralStationSide, coralStationPickupPosition);
  }

  /**
   * Logs all state information to AdvantageKit.
   * Records current selections, pathfinding states, and target paths.
   */
  public void logState() {
    // Log reef branch and level selections
    Logger.recordOutput(REEF_KEY + "/Branch", reefBranch.name());
    Logger.recordOutput(REEF_KEY + "/Level", reefLevel.name());

    // Log coral station selections
    Logger.recordOutput(CORAL_STATION_KEY + "/Side", coralStationSide.name());
    Logger.recordOutput(CORAL_STATION_KEY + "/Position", coralStationPickupPosition.name());

    // Log pathfinding status
    Logger.recordOutput(PATHFINDING_KEY + "/ToCoralStation", isPathfindingToCoralStation);
    Logger.recordOutput(PATHFINDING_KEY + "/ToReefBranch", isPathfindingToReefBranch);
    Logger.recordOutput(PATHFINDING_KEY + "/ActivePath", activePath);
    Logger.recordOutput(PATHFINDING_KEY + "/ReefBranchPath", getSelectedReefBranchPathName());
    Logger.recordOutput(PATHFINDING_KEY + "/CoralStationPath", getSelectedCoralStationPathName());
  }

  /**
   * Logs path data for debugging and visualization.
   * Records target poses and trajectory points for the current path.
   *
   * @param pathEndPose Optional end pose of the current path
   * @param pathTrajectory Optional list of trajectory poses
   */
  public void logPathData(Optional<Pose2d> pathEndPose, Optional<List<Pose2d>> pathTrajectory) {
    // Log target end pose if available
    pathEndPose.ifPresent(pose ->
            Logger.recordOutput(PATHFINDING_KEY + "/TargetPose", pose));

    // Log trajectory points if available
    pathTrajectory.ifPresent(poses ->
            Logger.recordOutput(PATHFINDING_KEY + "/Trajectory", poses.toArray(new Pose2d[0])));
  }

  /**
   * Logs target poses for both reef and coral station paths.
   * Records the end poses of current paths for debugging and visualization.
   *
   * @param reefBranchPose Optional end pose for the reef branch path
   * @param coralStationPose Optional end pose for the coral station path
   */
  public void logTargetPoses(Optional<Pose2d> reefBranchPose, Optional<Pose2d> coralStationPose) {
    reefBranchPose.ifPresent(pose ->
            Logger.recordOutput(REEF_KEY + "/TargetPose", pose));

    coralStationPose.ifPresent(pose ->
            Logger.recordOutput(CORAL_STATION_KEY + "/TargetPose", pose));
  }
}