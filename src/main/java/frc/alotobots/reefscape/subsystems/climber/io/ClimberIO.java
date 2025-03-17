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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for controlling the climber's hardware components. This interface defines methods for
 * controlling two servos and reading various sensor inputs: - A plunger servo that moves between
 * receive and plunge positions - A locking servo that secures the climbing cage - Cage limit
 * switches for detecting if the climbing mechanism is engaged
 */
public interface ClimberIO {

  /** AutoLogged class that contains all the inputs from the climber hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    /** Connection status of the servo hub */
    public boolean servoHubConnected = false;

    /** Current voltage of the servo hub */
    public Voltage servoHubVoltage = Volts.zero();

    /** Current draw of the servo hub */
    public Current servoHubCurrent = Amps.zero();

    /** Voltage supplied to the servos */
    public Voltage servoHubServoVoltage = Volts.zero();

    /** Enable state of the locking servo */
    public boolean lockingServoEnabled = false;

    /** Enable state of the plunger servo */
    public boolean plungerServoEnabled = false;

    /** State of the first cage limit switch */
    public boolean cageSwitch1 = false;

    /** State of the second cage limit switch */
    public boolean cageSwitch2 = false;

    /** Current pulse width of the plunger servo */
    public int plungerServoPulseWidth = 0;

    /** Current pulse width of the locking servo */
    public int lockingServoPulseWidth = 0;
  }

  /**
   * Gets the state of both cage limit switches.
   *
   * @return true if both cage switches are activated
   */
  public default boolean getCageSwitches() {
    return false;
  }

  /**
   * Updates the input values with current hardware states.
   *
   * @param inputs The ClimberIOInputs object to update
   */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Enables the locking servo. */
  public default void enableLockingServo() {}

  /** Disables the locking servo. */
  public default void disableLockingServo() {}

  /** Enables the plunger servo. */
  public default void enablePlungerServo() {}

  /** Disables the plunger servo. */
  public default void disablePlungerServo() {}

  /** Enables the elevator locking servo */
  public default void enableElevatorLockingServo() {}

  /** Disables the elevator locking servo */
  public default void disableElevatorLockingServo() {}

  /**
   * Sets the position of the plunger servo.
   *
   * @param angle The desired angle for the plunger servo
   */
  public default void setPlungerServoPosition(Angle angle) {}

  /**
   * Sets the locked state of the locking servo.
   *
   * @param lockingServoLocked true to lock, false to unlock
   */
  public default void setLockingServoLocked(boolean lockingServoLocked) {}

  public default void setElevatorLockingServoLocked(boolean elevatorLockingServoLocked) {}
}
