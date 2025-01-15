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
package frc.alotobots.reefscape.commands.scoring.reef.alignment.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.Constants;
import frc.alotobots.reefscape.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class BranchSelectionSubsystem extends SubsystemBase {
  private FieldConstants.ReefBranch selectedBranch;
  private FieldConstants.Level selectedLevel;

  public BranchSelectionSubsystem() {
    selectedBranch = FieldConstants.ReefBranch.A; // Default to first branch
    selectedLevel = FieldConstants.Level.L2; // Default to lowest level
  }

  @Override
  public void periodic() {
    Logger.recordOutput("BranchSelection/CurrentBranch", selectedBranch.name());
    Logger.recordOutput("BranchSelection/CurrentLevel", selectedLevel.name());
    Logger.recordOutput("BranchSelection/CurrentPath", getSelectedPathName());
  }

  public FieldConstants.ReefBranch getSelectedBranch() {
    return selectedBranch;
  }

  public FieldConstants.Level getSelectedLevel() {
    return selectedLevel;
  }

  public void cycleBranchForward() {
    int nextOrdinal = (selectedBranch.ordinal() + 1) % FieldConstants.ReefBranch.values().length;
    selectedBranch = FieldConstants.ReefBranch.values()[nextOrdinal];
  }

  public void cycleBranchBackward() {
    int nextOrdinal =
        (selectedBranch.ordinal() - 1 + FieldConstants.ReefBranch.values().length)
            % FieldConstants.ReefBranch.values().length;
    selectedBranch = FieldConstants.ReefBranch.values()[nextOrdinal];
  }

  public void cycleLevelUp() {
    int nextOrdinal = (selectedLevel.ordinal() + 1) % FieldConstants.Level.values().length;
    selectedLevel = FieldConstants.Level.values()[nextOrdinal];
  }

  public void cycleLevelDown() {
    int nextOrdinal =
        (selectedLevel.ordinal() - 1 + FieldConstants.Level.values().length)
            % FieldConstants.Level.values().length;
    selectedLevel = FieldConstants.Level.values()[nextOrdinal];
  }

  private String getSelectedPathName() {
    // Get the current state
    String pathName =
        String.format("BranchApproach_%s_%s", selectedBranch.name(), selectedLevel.name());

    Logger.recordOutput("BranchSelection/RequestedPath", pathName);
    return pathName;
  }

  public Command getPathfindingCommand() {
    return new InstantCommand(
        () -> {
          String pathName = getSelectedPathName();
          try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            Command pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    path, Constants.tunerConstants.getPathfindingConstraints());
            pathCommand.schedule(); // Schedule the pathfinding command immediately
          } catch (Exception e) {
            String errorMessage = "Failed to load path: " + pathName;
            Logger.recordOutput("BranchSelection/Error", errorMessage);
            new PrintCommand(errorMessage + " Not following path!").schedule();
          }
        });
  }

  // Commands for binding to controller
  public Command nextBranch() {
    return this.runOnce(this::cycleBranchForward);
  }

  public Command previousBranch() {
    return this.runOnce(this::cycleBranchBackward);
  }

  public Command nextLevel() {
    return this.runOnce(this::cycleLevelUp);
  }

  public Command previousLevel() {
    return this.runOnce(this::cycleLevelDown);
  }
}
