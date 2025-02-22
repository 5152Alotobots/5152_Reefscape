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
package frc.alotobots.reefscape.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.climber.io.ClimberIO;
import frc.alotobots.reefscape.subsystems.climber.io.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public boolean getCageSwitches() {
    return io.getCageSwitches();
  }

  public void enableServos() {
    io.enablePlungerServo();
    io.enableLockingServo();
  }

  public void disableServos() {
    io.disablePlungerServo();
    io.disableLockingServo();
  }

  public void setPlungerToAngle(Angle angle) {
    io.setPlungerServoPosition(angle);
  }

  public void setPlungerToPlunge() {
    io.setPlungerServoPosition(Degrees.of(0));
  }

  public void setPlungerToReceive() {
    io.setPlungerServoPosition(Degrees.of(180));
  }

  public void lockCage() {
    io.setLockingServoLocked(true);
  }

  public void unlockCage() {
    io.setLockingServoLocked(false);
  }
}
