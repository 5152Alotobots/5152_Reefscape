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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.Constants;
import frc.alotobots.reefscape.FieldConstants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class AutoCycleReefSubsystem extends SubsystemBase {
  @Getter
  private FieldConstants.ReefBranch selectedBranch;
  @Getter
  private FieldConstants.Level selectedLevel;

  public AutoCycleReefSubsystem() {
    selectedBranch = FieldConstants.ReefBranch.A; // Default to first branch
    selectedLevel = FieldConstants.Level.L2; // Default to lowest level
  }

  @Override
  public void periodic() {
    Logger.recordOutput("BranchSelection/CurrentBranch", selectedBranch.name());
    Logger.recordOutput("BranchSelection/CurrentLevel", selectedLevel.name());
    Logger.recordOutput("BranchSelection/CurrentPath", getSelectedPathName());
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

  /**
   * Gets the path name for approaching a specific reef branch at a given level.
   *
   * @param branch Target branch
   * @param level Target level
   * @return Path name for the approach
   */
  private String getReefPathName(FieldConstants.ReefBranch branch, FieldConstants.Level level) {
    return String.format("BranchApproach_%s_%s", branch, level);
  }

  public String getSelectedPathName() {
    return getReefPathName(selectedBranch, selectedLevel);
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
