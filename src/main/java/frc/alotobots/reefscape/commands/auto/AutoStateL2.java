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

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.reefscape.commands.groups.ElevatorWristRun;
import frc.alotobots.reefscape.commands.states.StateStowed;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeEjectThrough;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;

/**
 * Command sequence for placing game pieces on Level 2. The sequence: 1. Moves elevator and wrist to
 * L2 position simultaneously 2. Waits for release button confirmation 3. Runs eject through 4.
 * Returns to stowed position
 */
public class AutoStateL2 extends SequentialCommandGroup {
  /**
   * Creates a new StateL2 command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   * @param coralIntakeSubsystem The coral intake subsystem
   * @param coralIntakeReleaseTrigger The release button trigger
   */
  public AutoStateL2(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      Trigger coralIntakeReleaseTrigger) {
    addCommands(
        new LogCommand("State/State", "AUTO_L2"),
        new ElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.L2_PLACE,
            WristConstants.Setpoints.L2_PLACE),
        Commands.waitUntil(coralIntakeReleaseTrigger),
        new CoralIntakeEjectThrough(coralIntakeSubsystem, () -> EJECT_PERCENTAGE),
        new StateStowed(elevatorSubsystem, wristSubsystem).asProxy());
  }
}
