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
package frc.alotobots.reefscape.subsystems.elevator.io;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for elevator input/output operations. Handles the communication between the elevator
 * subsystem and its hardware implementation.
 */
public interface ElevatorIO {
  /**
   * Class containing all input values from elevator sensors and motors. Automatically logged
   * through {@link AutoLog} annotation.
   */
  @AutoLog
  public static class ElevatorIOInputs {
    /** Current PID slot */
    public int currentPidSlot = 0;

    /** Connection status of the left elevator motor */
    public boolean leftMotorConnected = false;

    /** Connection status of the right elevator motor */
    public boolean rightMotorConnected = false;

    /** Connection status of the CANRange position sensor */
    public boolean canrangeConnected = false;

    /** Top Software Limit */
    public boolean topLimit = false;

    /** Bottom Software Limit */
    public boolean bottomLimit = false;

    /** Current position of the left elevator motor */
    public Distance leftHeight = Meters.zero();

    /** Current position of the right elevator motor */
    public Distance rightHeight = Meters.zero();

    /** Current rotations of the left elevator motor */
    public Angle leftMotorAngle = Rotations.zero();

    /** Current rotations of the right elevator motor */
    public Angle rightMotorAngle = Rotations.zero();

    /** Current position reading from the CANCoder in meters */
    public Distance canrangeDistance = Meters.zero();

    /** Current velocity of the left elevator motor in meters per second */
    public LinearVelocity leftVelocity = MetersPerSecond.zero();

    /** Current velocity of the right elevator motor in meters per second */
    public LinearVelocity rightVelocity = MetersPerSecond.zero();

    /** Current acceleration of the left elevator motor in meters per second per second */
    public LinearAcceleration leftAcceleration = MetersPerSecondPerSecond.zero();

    /** Current acceleration of the right elevator motor in meters per second per second */
    public LinearAcceleration rightAcceleration = MetersPerSecondPerSecond.zero();

    /** Applied voltage to the left elevator motor */
    public Voltage leftAppliedVolts = Volts.zero();

    /** Applied voltage to the right elevator motor */
    public Voltage rightAppliedVolts = Volts.zero();

    /** Current draw of the left elevator motor in amperes */
    public Current leftCurrentAmps = Amps.zero();

    /** Current draw of the right elevator motor in amperes */
    public Current rightCurrentAmps = Amps.zero();
  }

  /**
   * Updates the input values from the elevator hardware. Called periodically to refresh sensor and
   * motor data.
   *
   * @param inputs The inputs object to update with the latest values
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the target position for the elevator using closed-loop control.
   *
   * @param position The desired position for the elevator
   */
  public default void setElevatorPosition(Distance position, int pidSlot) {}

  /**
   * Sets the target position for the elevator using closed-loop control & motion magic.
   *
   * @param position The desired position for the elevator
   */
  public default void setElevatorPositionMotionMagic(Distance position, int pidSlot) {}

  /**
   * Sets the target velocity for the elevator using closed-loop control.
   *
   * @param velocity The desired velocity for the elevator
   */
  public default void setElevatorVelocity(LinearVelocity velocity, int pidSlot) {}

  /**
   * Sets the elevator motors to run in open-loop control mode.
   *
   * @param percentOutput The desired motor output as a percentage (-1.0 to 1.0)
   */
  public default void setElevatorOpenLoop(double percentOutput) {}

  /**
   * Sets whether the elevator motors should brake when idle. Useful during disabled/init to prevent
   * gravity drift.
   *
   * @param brake true to enable brake mode, false for coast mode
   */
  public default void setElevatorBrakeMode(boolean brake) {}

  /** Stops all motors */
  public default void stop() {}
}
