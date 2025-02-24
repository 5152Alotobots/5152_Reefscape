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

public class PathfindToReef extends Command {
  private final AutoCycleSubsystem autoCycleSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private Command activePathCommand;
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  public PathfindToReef(
      AutoCycleSubsystem autoCycleSubsystem, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.autoCycleSubsystem = autoCycleSubsystem;
    this.pathPlannerManager = autoCycleSubsystem.getPathPlannerManager();
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    addRequirements(autoCycleSubsystem);
  }

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
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted && activePathCommand != null) {
      activePathCommand.cancel();
    }
    if (!interrupted) {
      autoCycleSubsystem.getState().setActivePathfindingCommand(null);
    }
  }

  @Override
  public boolean isFinished() {
    // Return true if either:
    // 1. Pathfinding is disabled
    // 2. Active path command exists and has finished
    return !autoCycleSubsystem.getState().isPathfindingEnabled()
        || (activePathCommand != null && !activePathCommand.isScheduled());
  }
}
