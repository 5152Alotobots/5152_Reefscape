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
package frc.alotobots.reefscape.subsystems.climber.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.*;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;

/**
 * Default command that runs the elevator at a specified velocity. This command takes a velocity
 * input (normalized between -1.0 and 1.0) and applies it to the elevator, scaled by the maximum
 * speed constant.
 */
public class Climb extends ParallelCommandGroup {
  /** The elevator subsystem this command controls. */
  private final ClimberSubsystem climberSubsystem;

  private final ElevatorSubsystem elevatorSubsystem;

  public Climb(ClimberSubsystem climberSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    addCommands(
        new InstantCommand(climberSubsystem::enableServos),
        new ElevatorRunToHeight(elevatorSubsystem, Meters.of(1.25)),
        new InstantCommand(climberSubsystem::setPlungerToReceive),
        new InstantCommand(climberSubsystem::unlockCage),
        new WaitUntilCommand(climberSubsystem::getCageSwitches),
        new InstantCommand(climberSubsystem::lockCage),
        new InstantCommand(climberSubsystem::setPlungerToPlunge),
        new WaitCommand(10),
        new InstantCommand(climberSubsystem::disableServos)
        // new InstantCommand(climberSubsystem::disableServos)
        // new ElevatorRunToHeight(elevatorSubsystem, MIN_HEIGHT)
        );
    addRequirements(climberSubsystem, elevatorSubsystem);
  }
}
