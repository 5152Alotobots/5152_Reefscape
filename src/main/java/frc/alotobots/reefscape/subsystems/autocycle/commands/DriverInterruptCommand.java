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

import static frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState.ActivePathfindingType.CORAL_STATION;
import static frc.alotobots.reefscape.subsystems.autocycle.util.AutoCycleState.ActivePathfindingType.REEF;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.autocycle.AutoCycleSubsystem;
import org.littletonrobotics.junction.Logger;

public class DriverInterruptCommand extends Command {
  private final AutoCycleSubsystem autoCycleSubsystem;

  public DriverInterruptCommand(AutoCycleSubsystem autoCycleSubsystem) {
    this.autoCycleSubsystem = autoCycleSubsystem;
  }

  @Override
  public void initialize() {
    autoCycleSubsystem.cancelActivePathfinding();
    autoCycleSubsystem.getState().setPaused(true);
    Logger.recordOutput("AutoCycle/DriverInterrupt/Status", "Started");
  }

  @Override
  public void end(boolean interrupted) {
    autoCycleSubsystem.getState().setPaused(false);
    // Resume pathfinding based on last active type
    switch (autoCycleSubsystem.getState().getLastActiveType()) {
      case REEF:
        new PathfindToReef(autoCycleSubsystem).schedule();
        break;
      case CORAL_STATION:
        new PathfindToCoralStation(autoCycleSubsystem).schedule();
        break;
      default:
        break;
    }
    Logger.recordOutput(
        "AutoCycle/DriverInterrupt/Status", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until trigger becomes false
  }
}
