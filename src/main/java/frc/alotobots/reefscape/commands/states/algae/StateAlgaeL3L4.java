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

import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE;

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
 * Command sequence for grabbing algae on Level 2/3. The sequence: 1. Moves elevator and wrist to
 * L2L3 position simultaneously 2. Runs intake
 */
public class StateAlgaeL3L4 extends SequentialCommandGroup {
  /**
   * Creates a new StateAlgaeL2L3 command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   * @param algaeIntakeSubsystem The algae intake subsystem
   */
  public StateAlgaeL3L4(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      AlgaeIntakeSubsystem algaeIntakeSubsystem,
      Trigger algaeIntakeReleaseTrigger) {
    addCommands(
        new LogCommand("State/State", "ALGAE_L2L3"),
        new ElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.ALGAE_L3L4_PICKUP,
            WristConstants.Setpoints.ALGAE_L3L4_PICKUP),
        new AlgaeIntakeIntakeOpenLoop(
            algaeIntakeSubsystem, algaeIntakeReleaseTrigger, () -> INTAKE_PERCENTAGE));
  }
}
