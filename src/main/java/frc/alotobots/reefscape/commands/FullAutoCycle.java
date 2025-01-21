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
package frc.alotobots.reefscape.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.reefscape.subsystems.autocycle.AutoCycleSubsystem;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToCoralStation;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToReef;

public class FullAutoCycle extends SequentialCommandGroup {
  public FullAutoCycle(AutoCycleSubsystem autoCycleSubsystem) {

    addRequirements(autoCycleSubsystem);
    addCommands(
        new ParallelCommandGroup(
            // Some logic to set arm to pickup position
            new PathfindToCoralStation(autoCycleSubsystem)),
        new ParallelCommandGroup(
            // Some logic to set arm to reef level position
            new PathfindToReef(autoCycleSubsystem)),
        autoCycleSubsystem.cycleReefBranchRight(false),
        new ParallelCommandGroup(
            // Some logic to set arm to pickup position
            new PathfindToCoralStation(autoCycleSubsystem)));
  }
}
