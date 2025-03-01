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
package frc.alotobots.reefscape.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorHoldHeight;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.commands.WristHoldAngle;

/**
 * A command group that schedules both elevator and wrist hold commands in parallel. This command is
 * designed to maintain the current position of both mechanisms simultaneously.
 *
 * <p>Note: This class is intended for autonomous routines only. Teleop operation uses a different
 * control method for these subsystems. Uses schedule commands to run until the next command starts.
 */
public class ElevatorWristHold extends ParallelCommandGroup {

  /**
   * Creates a new ElevatorWristHold command group.
   *
   * @param elevatorSubsystem The elevator subsystem to be controlled
   * @param wristSubsystem The wrist subsystem to be controlled
   */
  public ElevatorWristHold(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
    // This class should only be used for AUTOS. TELEOP handles it another way
    addCommands(
        new ScheduleCommand(new ElevatorHoldHeight(elevatorSubsystem)),
        new ScheduleCommand(new WristHoldAngle(wristSubsystem)));
  }
}
