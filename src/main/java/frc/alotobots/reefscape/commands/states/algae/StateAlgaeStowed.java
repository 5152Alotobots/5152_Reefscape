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
package frc.alotobots.reefscape.commands.states.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.reefscape.commands.groups.ElevatorWristRun;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;

/** Command sequence for stowing with algae. */
public class StateAlgaeStowed extends SequentialCommandGroup {
  /**
   * Creates a new StateAlgaeStowed command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   */
  public StateAlgaeStowed(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    addCommands(
        new LogCommand("State/State", "ALGAE_STOWED"),
        new ElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.ALGAE_STOWED,
            WristConstants.Setpoints.ALGAE_STOWED));
  }
}
