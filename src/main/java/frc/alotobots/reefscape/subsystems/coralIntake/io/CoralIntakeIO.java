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
package frc.alotobots.reefscape.subsystems.coralIntake.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {
    public boolean motorConnected = false;
    public boolean intakeOccupied = false;

    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage motorAppliedVolts = Volts.zero();
    public Current motorCurrentAmps = Amps.zero();
  }

  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  public default void setIntakeVelocity(AngularVelocity velocity, int pidSlot) {}

  public default void setIntakeOpenLoop(double percentOutput) {}

  public default boolean getIntakeOccupied() { return false; }

  public default void stop() {}
}
