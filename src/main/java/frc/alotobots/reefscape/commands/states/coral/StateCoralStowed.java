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
package frc.alotobots.reefscape.commands.states.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.reefscape.commands.groups.SequentialWristElevatorRun;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;

/**
 * Command for moving the robot's mechanisms to their stowed positions. Moves the wrist first, then
 * the elevator to their positions
 */
public class StateCoralStowed extends SequentialCommandGroup {
  /**
   * Creates a new StateCoralStowed command. Runs both elevator and wrist to stowed position. Wrist
   * moves first, then elevator. Cedes control back to the default command via usage of ProxyCommand
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   */
  public StateCoralStowed(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    addCommands(
        new LogCommand("State/State", "CORAL_STOWED"),
        new SequentialWristElevatorRun(
            wristSubsystem,
            elevatorSubsystem,
            ElevatorConstants.Setpoints.CORAL_STOWED,
            WristConstants.Setpoints.STOWED));
  }
}
