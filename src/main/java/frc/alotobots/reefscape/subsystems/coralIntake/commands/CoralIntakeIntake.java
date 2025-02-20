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
package frc.alotobots.reefscape.subsystems.coralIntake.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import java.util.function.DoubleSupplier;

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Limits.MAX_OPEN_LOOP_INTAKE_PERCENTAGE;
import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE;

/**
 * Command that runs the intake to collect game pieces at a specified speed. Automatically ends
 * when a game piece is detected by the intake sensor.
 */
public class CoralIntakeIntake extends Command {
    /** The coral intake subsystem being controlled. */
    private final CoralIntakeSubsystem coralIntakeSubsystem;

    /** The input for controlling intake speed. */
    private final DoubleSupplier input;

    /**
     * Creates a new CoralIntakeIntake command.
     *
     * @param coralIntakeSubsystem The intake subsystem to control
     * @param input Supplier for the intake speed (0.0 to MAX_OPEN_LOOP_INTAKE_PERCENTAGE)
     */
    public CoralIntakeIntake(CoralIntakeSubsystem coralIntakeSubsystem, DoubleSupplier input) {
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.input = input;
        addRequirements(coralIntakeSubsystem);
    }

    /**
     * Runs the intake motors at the supplied speed.
     */
    @Override
    public void execute() {
        double adjustedOutput = MathUtil.clamp(input.getAsDouble(), 0, MAX_OPEN_LOOP_INTAKE_PERCENTAGE);
        coralIntakeSubsystem.runAtPercentOutput(adjustedOutput);
    }

    /**
     * Called when the command ends or is interrupted. Stops the intake motors.
     *
     * @param interrupted true if the command was interrupted, false if it completed normally
     */
    @Override
    public void end(boolean interrupted) {
        coralIntakeSubsystem.stop();
    }

    /**
     * Determines if the command has finished. Returns true once a game piece is detected.
     *
     * @return true if a game piece is detected in the intake
     */
    @Override
    public boolean isFinished() {
        return coralIntakeSubsystem.isIntakeOccupied();
    }
}