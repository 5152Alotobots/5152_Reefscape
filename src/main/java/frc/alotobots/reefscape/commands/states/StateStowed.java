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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.commands.WristRunToAngle;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;
import frc.alotobots.util.commandbase.ReleasingSequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

/**
 * Command for moving the robot's mechanisms to their stowed positions. Moves the wrist first, then
 * the elevator to their positions
 */
public class StateStowed extends ReleasingSequentialCommandGroup {
  /**
   * Creates a new StateStowed command.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param wristSubsystem The wrist subsystem
   */
  public StateStowed(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    addCommands(
        new InstantCommand(() -> Logger.recordOutput("State/State", "STOWED")),
        new WristRunToAngle(wristSubsystem, WristConstants.Setpoints.STOWED),
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.STOWED));
  }
}
