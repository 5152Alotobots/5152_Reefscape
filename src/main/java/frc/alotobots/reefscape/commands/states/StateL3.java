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
package frc.alotobots.reefscape.commands.states;

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.reefscape.commands.groups.ParallelElevatorWristRun;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeEjectThrough;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Command sequence for placing game pieces on Level 3. The sequence: 1. Moves elevator and wrist to
 * L3 position simultaneously 2. Waits for release button confirmation 3. Runs eject through 4.
 * Returns to stowed position
 */
public class StateL3 extends SequentialCommandGroup {
  /**
   * Creates a new StateL3 command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   * @param coralIntakeSubsystem The coral intake subsystem
   * @param coralIntakeReleaseTrigger The release button trigger
   */
  public StateL3(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      Trigger coralIntakeReleaseTrigger) {
    addCommands(
            new LogCommand("State/State", "L3"),
        new ParallelElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.L3_PLACE,
            WristConstants.Setpoints.L3_PLACE),
        Commands.waitUntil(coralIntakeReleaseTrigger),
        new CoralIntakeEjectThrough(coralIntakeSubsystem, () -> EJECT_PERCENTAGE),
        new StateStowed(elevatorSubsystem, wristSubsystem).asProxy());
  }
}
