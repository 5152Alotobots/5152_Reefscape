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
package frc.alotobots.reefscape.subsystems.wrist.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {

    public int pidSlot = 0;
    public boolean motorConnected = false;
    public boolean cancoderConnected = false;
    public boolean topLimit = false;
    public boolean bottomLimit = false;

    public Angle mechanismAngle = Rotations.zero();
    public AngularVelocity rotationVelocity = RotationsPerSecond.zero();
    public Voltage motorAppliedVolts = Volts.zero();
    public Current motorCurrent = Amps.zero();
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setWristPosition(Angle rotation, int pidSlot) {}

  public default void setWristOpenLoop(double percentOutput) {}

  public default void stop() {}
}
