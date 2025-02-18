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
package frc.alotobots.reefscape.subsystems.climber.io;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public boolean servoHubConnected = false;
    public boolean plungerServoPowered = false;
    public boolean lockingServoPowered = false;

    public boolean plungerServoEnabled = false;
    public boolean lockingServoEnabled = false;

    public Angle plungerServoPosition = Degrees.zero();
    public Angle lockingServoPosition = Degrees.zero();
  }

  public void togglePlungerServoEnabled();
  public void toggleLockingServoEnabled();
  public void setPlungerServoPosition();

  public void setLockingServoPosition();
}
