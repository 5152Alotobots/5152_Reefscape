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

import edu.wpi.first.math.util.Units.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public boolean servoHubConnected = false;

    public int plungerServoPosition = 0;
    public int lockingServoPosition = 0;
  }

  public void updateInputs(ClimberIOInputs inputs);

  public void togglePlungerServoEnabled();

  public void toggleLockingServoEnabled();

  public void setPlungerServoPosition();

  public void setLockingServoPosition();
}
