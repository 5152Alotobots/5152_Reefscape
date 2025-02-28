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
 * Command sequence for placing game pieces on Level 1. The sequence: 1. Moves elevator and wrist to
 * L1 position simultaneously 2. Waits for release button confirmation 3. Runs eject 4. Returns to
 * stowed position
 */
public class AutoStateL1 extends SequentialCommandGroup {
  /**
   * Creates a new StateL1 command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   */
  public AutoStateL1(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    addCommands(
        new LogCommand("State/State", "AUTO_L1"),
        new ElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.L1_PLACE,
            WristConstants.Setpoints.L1_PLACE),
        new ElevatorWristHold(elevatorSubsystem, wristSubsystem));
  }
}
