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

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.reefscape.commands.groups.ParallelElevatorWristRun;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeIntake;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;

/**
 * Command sequence for intaking from the coral station. The sequence: 1. Moves elevator and wrist
 * to coral station position simultaneously 2. Waits for release button confirmation 3. Runs intake
 * 4. Returns to stowed position
 */
public class StateCoralCoralStation extends SequentialCommandGroup {
  /**
   * Creates a new StateCoralStation command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   * @param coralIntakeSubsystem The coral intake subsystem
   */
  public StateCoralCoralStation(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem) {
    addCommands(
        new LogCommand("State/State", "CORAL_CORAL_STATION"),
        new ParallelElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.CORAL_CORAL_STATION,
            WristConstants.Setpoints.CORAL_CORAL_STATION),
        new CoralIntakeIntake(coralIntakeSubsystem, () -> INTAKE_PERCENTAGE),
        new StateCoralStowed(elevatorSubsystem, wristSubsystem).asProxy());
  }
}
