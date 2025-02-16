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
package frc.alotobots.reefscape.subsystems.coralIntake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.coralIntake.io.CoralIntakeIO;
import frc.alotobots.reefscape.subsystems.coralIntake.io.CoralIntakeIOInputsAutoLogged;

public class CoralIntakeSubsystem extends SubsystemBase {
  private CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();
  private CoralIntakeIO io;

  public CoralIntakeSubsystem(CoralIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void runAtPercentOutput(double percentOutput) {
    io.setIntakeOpenLoop(percentOutput);
  }

  public void runAtPercentOutputWithLimits(double percentOutput) {
    if (io.getIntakeOccupied()) {
      io.setIntakeOpenLoop(Math.min(0, percentOutput));
    } else {
      io.setIntakeOpenLoop(percentOutput);
    }
  }

  public void stop() {
    io.stop();
  }
}


