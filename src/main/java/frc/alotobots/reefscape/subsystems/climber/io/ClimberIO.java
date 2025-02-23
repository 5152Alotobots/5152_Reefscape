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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public boolean servoHubConnected = false;
    public Voltage servoHubVoltage = Volts.zero();
    public Current servoHubCurrent = Amps.zero();

    public Voltage servoHubServoVoltage = Volts.zero();

    public boolean lockingServoEnabled = false;
    public boolean plungerServoEnabled = false;

    public boolean cageSwitch1 = false;
    public boolean cageSwitch2 = false;

    public int plungerServoPulseWidth = 0;
    public int lockingServoPulseWidth = 0;
  }

  boolean getCageSwitches();

  public void updateInputs(ClimberIOInputs inputs);

  public void enableLockingServo();

  public void disableLockingServo();

  public void enablePlungerServo();

  public void disablePlungerServo();

  public void setPlungerServoPosition(Angle angle);

  public void setLockingServoLocked(boolean lockingServoLocked);
}
