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

/**
 * Interface defining hardware abstraction for the coral intake mechanism. Provides methods for
 * controlling the intake motors and reading sensor data.
 */
public interface CoralIntakeIO {

  /**
   * Input data structure for the coral intake hardware interface. Contains sensor readings and
   * motor controller states.
   */
  @AutoLog
  public static class CoralIntakeIOInputs {
    /** Connection status of the intake motor controller */
    public boolean motorConnected = false;

    /** Connection status of the CANrange sensor */
    public boolean canRangeConnected = false;

    /** Current game piece detection status */
    public boolean intakeOccupied = false;

    /** Current angular velocity of the intake motor */
    public AngularVelocity motorVelocity = RotationsPerSecond.zero();

    /** Current voltage being applied to the intake motor */
    public Voltage motorAppliedVolts = Volts.zero();

    /** Current being drawn by the intake motor */
    public Current motorCurrentAmps = Amps.zero();
  }

  /**
   * Updates the input values with the latest hardware state.
   *
   * @param inputs The input object to update with new values
   */
  public default void updateInputs(CoralIntakeIOInputs inputs) {}

  /**
   * Sets the intake motor to run at a specified velocity using closed-loop control.
   *
   * @param velocity Target velocity for the intake
   * @param pidSlot PID slot to use for velocity control
   */
  public default void setIntakeVelocity(AngularVelocity velocity, int pidSlot) {}

  /**
   * Sets the intake motor to run at a specified percentage of full power.
   *
   * @param percentOutput Motor output percentage (-1.0 to 1.0)
   */
  public default void setIntakeOpenLoop(double percentOutput) {}

  /**
   * Gets the current game piece detection status.
   *
   * @return true if a game piece is detected in the intake
   */
  public default boolean getIntakeOccupied() {
    return false;
  }

  /** Stops all intake motor movement. */
  public default void stop() {}
}
