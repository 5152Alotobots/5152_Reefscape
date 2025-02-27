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
package frc.alotobots.reefscape.subsystems.autocycle.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.reefscape.subsystems.autocycle.AutoCycleSubsystem;
import frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState.ActivePathfindingType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command to pathfind to the currently selected reef branch position. Uses PathPlanner's
 * pathfinding capabilities with driver override to navigate to the appropriate reef branch approach
 * path.
 */
public class PathfindToReef extends Command {
  private final AutoCycleSubsystem autoCycleSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private Command activePathCommand;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  /**
   * Creates a new PathfindToReef command.
   *
   * @param autoCycleSubsystem The AutoCycle subsystem that manages path selections
   * @param chassisSpeedsSupplier A supplier for driver input as chassis speeds
   */
  public PathfindToReef(
      AutoCycleSubsystem autoCycleSubsystem, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.autoCycleSubsystem = autoCycleSubsystem;
    this.pathPlannerManager = autoCycleSubsystem.getPathPlannerManager();
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    addRequirements(autoCycleSubsystem);
  }

  /**
   * Called when the command is initially scheduled. Begins pathfinding to the selected reef branch
   * if pathfinding is enabled.
   */
  @Override
  public void initialize() {
    if (autoCycleSubsystem.getState().isPathfindingEnabled()) {
      autoCycleSubsystem.getState().setActivePathfinding(ActivePathfindingType.REEF);
      activePathCommand =
          pathPlannerManager.getPathfindThenFollowPathCommandWithOverride(
              autoCycleSubsystem.getState().getSelectedReefBranchPathName(),
              chassisSpeedsSupplier,
              true);
      autoCycleSubsystem.getState().setActivePathfindingCommand(activePathCommand);
      activePathCommand.schedule();

      Logger.recordOutput(
          "PathfindToReef/PathName", autoCycleSubsystem.getState().getSelectedReefBranchPathName());
    }
  }

  /**
   * Called once when the command ends or is interrupted. Ensures proper cleanup of active path
   * commands.
   *
   * @param interrupted Whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    if (interrupted && activePathCommand != null) {
      activePathCommand.cancel();
    }
    if (!interrupted) {
      autoCycleSubsystem.getState().setActivePathfindingCommand(null);
    }

    Logger.recordOutput("PathfindToReef/Completed", !interrupted);
  }

  /**
   * Returns whether this command has finished. The command finishes when either pathfinding is
   * disabled or the active path command completes.
   *
   * @return True if the command is finished
   */
  @Override
  public boolean isFinished() {
    // Return true if either:
    // 1. Pathfinding is disabled
    // 2. Active path command exists and has finished
    return !autoCycleSubsystem.getState().isPathfindingEnabled()
        || (activePathCommand != null && !activePathCommand.isScheduled());
  }
}
