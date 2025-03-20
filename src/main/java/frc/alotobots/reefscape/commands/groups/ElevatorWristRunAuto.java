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

import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Setpoints.CRUISE;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.commands.WristRunToAngle;

public class ElevatorWristRunAuto extends SequentialCommandGroup {
  /**
   * Creates a new ElevatorWristRun command group that moves both the elevator and wrist to
   * specified positions one after the other. Runs the wrist first, then the elevator. Proxies the
   * commands to cede back to default command after runtime automatically.
   *
   * @param elevatorSubsystem The elevator subsystem
   * @param elevatorHeight Target height for the elevator
   * @param wristSubsystem The wrist subsystem
   * @param wristAngle Target angle for the wrist
   */
  public ElevatorWristRunAuto(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      Distance elevatorHeight,
      Angle wristAngle) {

    addCommands(
        new ParallelCommandGroup(
            new WristRunToAngle(wristSubsystem, CRUISE).asProxy(),
            new ElevatorRunToHeight(elevatorSubsystem, elevatorHeight).asProxy()),
        new WristRunToAngle(wristSubsystem, wristAngle).asProxy());
  }
}
