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

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.*;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunAtClimbVelocity;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;

import java.util.function.DoubleSupplier;

public class Climb extends SequentialCommandGroup {

  public Climb(ClimberSubsystem climberSubsystem, ElevatorSubsystem elevatorSubsystem, DoubleSupplier input) {

    addCommands(
        new InstantCommand(climberSubsystem::enableServos),
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.CLIMB),
        new InstantCommand(climberSubsystem::setPlungerToReceive),
        new InstantCommand(climberSubsystem::unlockCage),
        new WaitUntilCommand(climberSubsystem::getCageSwitches),
        new InstantCommand(climberSubsystem::lockCage),
        new InstantCommand(climberSubsystem::setPlungerToPlunge),
            new ElevatorRunAtClimbVelocity(elevatorSubsystem, input)
        );
    addRequirements(climberSubsystem, elevatorSubsystem);
  }
}
