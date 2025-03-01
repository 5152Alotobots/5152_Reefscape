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
package frc.alotobots.reefscape.subsystems.algaeintake.commands;

import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeConstants.Limits.MAX_INTAKE_VELOCITY;
import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeConstants.Limits.MAX_OPEN_LOOP_INTAKE_PERCENTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.alotobots.reefscape.subsystems.algaeintake.AlgaeIntakeSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command that runs the algae intake to collect game pieces by pulling them inward using positive
 * motor output. Automatically ends when a game piece is detected by the intake sensor. The speed is
 * clamped to the maximum allowed intake percentage.
 */
public class AlgaeIntakeIntakeAtVelocity extends Command {
    /** The algae intake subsystem being controlled */
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    private final Trigger stop;

    /**
     * Creates a new AlgaeIntakeIntakeAtVelocity command.
     *
     * @param algaeIntakeSubsystem The intake subsystem to control
     * @param stop the cancel trigger
     */
    public AlgaeIntakeIntakeAtVelocity(
            AlgaeIntakeSubsystem algaeIntakeSubsystem, Trigger stop) {
        this.stop = stop;
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        addRequirements(algaeIntakeSubsystem);
    }

    /**
     * Runs the intake motors at the supplied speed to pull inward, clamped to safe limits.
     */
    @Override
    public void initialize() {
        algaeIntakeSubsystem.runToTargetVelocity(MAX_INTAKE_VELOCITY);
    }

    /**
     * Called when the command ends or is interrupted. Stops the intake motors.
     *
     * @param interrupted true if the command was interrupted, false if it completed normally
     */
    @Override
    public void end(boolean interrupted) {
        algaeIntakeSubsystem.stop();
    }

    /**
     * Determines if the command has finished. Returns true once a game piece is detected.
     *
     * @return true if a game piece is detected in the intake
     */
    @Override
    public boolean isFinished() {
        return stop.getAsBoolean() || algaeIntakeSubsystem.isIntakeOccupied();
    }
}
