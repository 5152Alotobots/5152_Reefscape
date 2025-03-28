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
package frc.alotobots.reefscape.subsystems.algaeintake.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface defining hardware abstraction for the algae intake mechanism. Provides methods for
 * controlling the intake motors (two Neo 550s) and reading sensor data.
 */
public interface AlgaeIntakeIO {

  /**
   * Input data structure for the algae intake hardware interface. Contains sensor readings and
   * motor controller states for both Neo 550 motors.
   */
  @AutoLog
  public static class AlgaeIntakeIOInputs {
    /** Connection status of the left intake motor controller */
    public boolean leftMotorConnected = false;

    /** Connection status of the right intake motor controller */
    public boolean rightMotorConnected = false;

    /** Connection status of the CANrange sensor */
    public boolean canRangeConnected = false;

    /** Current game piece detection status from CANrange */
    public boolean canRangeInProximity = false;

    /** Current angular velocity of the left intake motor */
    public AngularVelocity leftVelocity = RotationsPerSecond.zero();

    /** Current angular velocity of the right intake motor */
    public AngularVelocity rightVelocity = RotationsPerSecond.zero();

    /** Current voltage being applied to the left intake motor */
    public Voltage leftAppliedVolts = Volts.zero();

    /** Current voltage being applied to the right intake motor */
    public Voltage rightAppliedVolts = Volts.zero();

    /** Current being drawn by the left intake motor */
    public Current leftCurrentAmps = Amps.zero();

    /** Current being drawn by the right intake motor */
    public Current rightCurrentAmps = Amps.zero();

    /** Left motor temperature */
    public Temperature leftMotorTemp = Celsius.zero();

    /** Right motor temperature */
    public Temperature rightMotorTemp = Celsius.zero();

    /** Left motor acceleration */
    public AngularAcceleration leftAcceleration = RadiansPerSecondPerSecond.zero();

    /** Right motor acceleration */
    public AngularAcceleration rightAcceleration = RadiansPerSecondPerSecond.zero();

    /** Current PID slot */
    public int currentPidSlot = 0;
  }

  /**
   * Updates the input values with the latest hardware state.
   *
   * @param inputs The input object to update with new values
   */
  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  /**
   * Sets the intake motors to run at a specified percentage of full power.
   *
   * @param percentOutput Motor output percentage (-1.0 to 1.0) Positive values are for intake,
   *     negative for outtake
   */
  public default void setAlgaeIntakeOpenLoop(double percentOutput) {}

  public default void setAlgaeIntakeVelocity(AngularVelocity velocity, int pidSlot) {}

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

  /** Sets the brake mode */
  public default void setAlgaeIntakeBrakeMode(boolean brake) {}
}
