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
package frc.alotobots;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.reefscape.commands.auto.*;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeEject;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeEjectThrough;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeIntake;
import frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import java.util.HashMap;
import java.util.Map;

/** Registers and manages named commands for autonomous routines. */
public class AutoNamedCommands {
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final CoralIntakeSubsystem coralIntakeSubsystem;
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** Constructs command registration manager with required subsystems. */
  public AutoNamedCommands(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      CoralIntakeSubsystem coralIntakeSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem) {

    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.coralIntakeSubsystem = coralIntakeSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;

    registerCommands();
  }

  /** Registers all available autonomous commands with PathPlanner. */
  public void registerCommands() {
    Map<String, Command> commands = new HashMap<>();

    // Auto states
    commands.put("AutoStateL1", new AutoStateL1(elevatorSubsystem, wristSubsystem));
    commands.put("AutoStateL2", new AutoStateL2(elevatorSubsystem, wristSubsystem));
    commands.put("AutoStateL3", new AutoStateL3(elevatorSubsystem, wristSubsystem));
    commands.put("AutoStateL4", new AutoStateL4(elevatorSubsystem, wristSubsystem));
    commands.put("AutoStateStowed", new AutoStateStowed(elevatorSubsystem, wristSubsystem));
    commands.put(
        "AutoStateCoralStation", new AutoStateCoralStation(elevatorSubsystem, wristSubsystem));
    commands.put("AutoStopWithX", new InstantCommand(swerveDriveSubsystem::stopWithX));
    // Coral Intake Commands
    commands.put(
        "CoralIntakeEject",
        new CoralIntakeEject(
            coralIntakeSubsystem, () -> CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE));
    commands.put(
        "CoralIntakeEjectThrough",
        new CoralIntakeEjectThrough(
            coralIntakeSubsystem, () -> CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE));
    commands.put(
        "CoralIntakeIntake",
        new CoralIntakeIntake(
            coralIntakeSubsystem, () -> CoralIntakeConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE));
    commands.put(
        "CoralIntakeEjectPassive",
        new CoralIntakeEject(
            coralIntakeSubsystem,
            () -> (CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE * .75)));
    commands.put(
        "CoralIntakeEjectThroughPassive",
        new CoralIntakeEjectThrough(
            coralIntakeSubsystem,
            () -> (CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE * .75)));
    commands.put(
        "CoralIntakeEjectAggressive",
        new CoralIntakeEject(
            coralIntakeSubsystem,
            () -> (CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE * 1.5)));
    commands.put(
        "CoralIntakeEjectThroughAggressive",
        new CoralIntakeEjectThrough(
            coralIntakeSubsystem,
            () -> (CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE * 1.5)));

    NamedCommands.registerCommands(commands);
  }
}
