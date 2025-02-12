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

package frc.alotobots.reefscape.subsystems.wrist.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;



public class DefaultWristCommand extends Command {
    private final WristSubsystem wristSubsystem;
    private final DoubleSupplier input;

    public DefaultWristCommand(DoubleSupplier input, WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        this.input = input;

        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        wristSubsystem.runAtPercentOutput(input.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.stop();
    }
    
}
