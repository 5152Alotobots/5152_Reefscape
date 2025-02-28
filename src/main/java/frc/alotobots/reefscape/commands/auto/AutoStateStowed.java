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
package frc.alotobots.reefscape.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.reefscape.commands.groups.ElevatorWristHold;
import frc.alotobots.reefscape.commands.groups.ElevatorWristRun;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;

/**
 * Command for moving the robot's mechanisms to their stowed positions. Moves the wrist first, then
 * the elevator to their positions
 */
public class AutoStateStowed extends SequentialCommandGroup {
  /**
   * Creates a new StateStowed command. Runs both elevator and wrist to stowed position. Wrist moves
   * first, then elevator. Cedes control back to the default command via usage of ProxyCommand
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   */
  public AutoStateStowed(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    addCommands(
        new LogCommand("State/State", "AUTO_CORAL_STOWED"),
        new ElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.CORAL_STOWED,
            WristConstants.Setpoints.CORAL_STOWED),
        new ElevatorWristHold(elevatorSubsystem, wristSubsystem));
  }
}
