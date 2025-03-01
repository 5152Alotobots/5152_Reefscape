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

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.reefscape.commands.groups.ElevatorWristRun;
import frc.alotobots.reefscape.subsystems.algaeintake.AlgaeIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.algaeintake.commands.AlgaeIntakeIntakeOpenLoop;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;

/**
 * Command sequence for picking Algae from the ground. The sequence: 1. Moves elevator and wrist to
 * processor position simultaneously 2. Waits for release button confirmation 3. Runs eject 4.
 * Returns to stowed position
 */
public class StateAlgaeGround extends SequentialCommandGroup {
  /**
   * Creates a new StateAlgaeProcessor command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   * @param algaeIntakeSubsystem The algae intake subsystem
   * @param algaeIntakeReleaseTrigger The release button trigger
   */
  public StateAlgaeGround(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      AlgaeIntakeSubsystem algaeIntakeSubsystem,
      Trigger algaeIntakeReleaseTrigger) {
    addCommands(
        new LogCommand("State/State", "ALGAE_PROCESSOR"),
        new ElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.ALGAE_PROCESSOR,
            WristConstants.Setpoints.ALGAE_PROCESSOR,
            true),
        new AlgaeIntakeIntakeOpenLoop(
            algaeIntakeSubsystem, algaeIntakeReleaseTrigger, () -> INTAKE_PERCENTAGE),
        new StateAlgaeStowed(elevatorSubsystem, wristSubsystem).asProxy());
  }
}
