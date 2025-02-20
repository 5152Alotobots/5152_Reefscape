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

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Setpoints.OpenLoop.OUTTAKE_PERCENTAGE;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeOuttakeThrough;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.commands.WristRunToAngle;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Command sequence for placing game pieces on Level 4. The sequence: 1. Moves elevator and wrist to
 * L4 position simultaneously 2. Waits for release button confirmation 3. Runs outtake through 4.
 * Returns to stowed position
 */
public class StateL4 extends SequentialCommandGroup {
  /**
   * Creates a new StateL4 command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   * @param coralIntakeSubsystem The coral intake subsystem
   * @param coralIntakeReleaseTrigger The release button trigger
   */
  public StateL4(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      Trigger coralIntakeReleaseTrigger) {
    addCommands(
        new ParallelCommandGroup(
                new InstantCommand(() -> Logger.recordOutput("State/State", "L4")),
            new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.L4_PLACE),
            new WristRunToAngle(wristSubsystem, WristConstants.Setpoints.L4_PLACE)),
        Commands.waitUntil(coralIntakeReleaseTrigger),
        new CoralIntakeOuttakeThrough(coralIntakeSubsystem, () -> OUTTAKE_PERCENTAGE),
        new StateStow(elevatorSubsystem, wristSubsystem));
  }
}
