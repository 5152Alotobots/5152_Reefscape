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
package frc.alotobots.reefscape.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIO;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import frc.alotobots.reefscape.util.GameElement;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.MAX_HEIGHT;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.MIN_HEIGHT;

public class ElevatorSubsystem extends SubsystemBase {
  ElevatorIO io;
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  Supplier<GameElement> elementInIntake;

  public ElevatorSubsystem(ElevatorIO io, Supplier<GameElement> elementInIntake) {
    this.io = io;
    this.elementInIntake = elementInIntake;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void runToTargetPosition(Distance height) {
    Distance adjustedHeight = Meters.of(MathUtil.clamp(height.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters)));
    switch (elementInIntake.get()) {
      case NONE:
        io.setElevatorPosition(adjustedHeight, ElevatorConstants.PIDSlot.NONE);
        break;
      case CORAL:
        io.setElevatorPosition(adjustedHeight, ElevatorConstants.PIDSlot.CORAL);
        break;
      case ALGAE:
        io.setElevatorPosition(adjustedHeight, ElevatorConstants.PIDSlot.ALGAE);
        break;
    }
  }

  public void runAtPercentOutput(double percentOutput) {
    io.setElevatorOpenLoop(percentOutput);
  }

  public void stop() {
    io.stop();
    io.setElevatorBrakeMode(true);
  }

}
