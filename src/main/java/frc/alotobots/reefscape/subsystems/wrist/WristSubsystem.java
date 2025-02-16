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
package frc.alotobots.reefscape.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.MAX_ANGLE;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.MIN_ANGLE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIO;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private WristIO io;

  public WristSubsystem(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  public void runToTargetAngle(Angle angle) {
    Angle adjustedAngle =
        Degrees.of(MathUtil.clamp(angle.in(Degrees), MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees)));
    io.setWristPosition(adjustedAngle, 0);
  }

  public void runAtPercentOutput(double percentOutput) {
    io.setWristOpenLoop(percentOutput);
  }

  public void stop() {
    io.stop();
  }
}
