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

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.BLING_NOTIFICATION_TIME;

import edu.wpi.first.wpilibj2.command.*;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.library.subsystems.bling.commands.BlingCageSwitchActive;
import frc.alotobots.library.subsystems.bling.commands.BlingClimberReady;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunAtClimbVelocity;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import java.util.function.DoubleSupplier;

/**
 * A sequential command group that handles the climbing sequence. This command coordinates the
 * elevator and climber subsystems to perform a climbing operation.
 */
public class Climb extends SequentialCommandGroup {

  /**
   * Creates a new Climb command.
   *
   * @param climberSubsystem The climber subsystem to control
   * @param elevatorSubsystem The elevator subsystem to control
   * @param blingSubsystem The bling subsystem to control
   * @param input The input supplier for controlling climb velocity
   */
  public Climb(
      ClimberSubsystem climberSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BlingSubsystem blingSubsystem,
      DoubleSupplier input) {
    addCommands(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.CLIMB).asProxy(),
        new InstantCommand(climberSubsystem::setPlungerToReceive),
        new InstantCommand(climberSubsystem::enableServos),
        new InstantCommand(climberSubsystem::unlockCage),
        new ParallelDeadlineGroup(
                new WaitUntilCommand(climberSubsystem::getCageSwitches),
                new BlingCageSwitchActive(blingSubsystem, climberSubsystem::getCageSwitchesRaw))
            .asProxy(),
        new InstantCommand(climberSubsystem::lockCage),
        new InstantCommand(climberSubsystem::setPlungerToPlunge),
        new ScheduleCommand(
                new BlingClimberReady(blingSubsystem).withTimeout(BLING_NOTIFICATION_TIME))
            .asProxy(),
        new ParallelCommandGroup(
            new ElevatorRunAtClimbVelocity(elevatorSubsystem, input).asProxy(),
            new SequentialCommandGroup(
                new WaitUntilCommand(elevatorSubsystem::isAtMinClimbHeight),
                new InstantCommand(climberSubsystem::lockElevator))));
    addRequirements(climberSubsystem);
  }
}
