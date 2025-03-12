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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.library.commands.util.LogCommand;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.reefscape.commands.groups.ElevatorWristRun;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;

/**
 * Command sequence for removing algae from level 2
 */
public class StateAlgaeRemoveL3 extends SequentialCommandGroup {
  /**
   * Creates a new StateL2 command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   * @param coralIntakeSubsystem The coral intake subsystem
   * @param blingSubsystem The bling subsystem
   * @param coralIntakeReleaseTrigger The release button trigger
   */
  public StateAlgaeRemoveL3(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      BlingSubsystem blingSubsystem,
      Trigger coralIntakeReleaseTrigger) {
    addCommands(
        new LogCommand("State/State", "ALGAE_L3"),
        new ElevatorWristRun(
            elevatorSubsystem,
            wristSubsystem,
            ElevatorConstants.Setpoints.ALGAE_L3_REMOVAL,
            WristConstants.Setpoints.ALGAE_L3_REMOVAL));
  }
}
