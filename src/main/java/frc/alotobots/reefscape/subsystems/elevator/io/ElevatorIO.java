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
    /* Current PID slot */
    public int currentPidSlot = 0;

    /* Connection status of the left elevator motor */
    public boolean leftMotorConnected = false;
    public boolean rightMotorConnected = false;

    /* Software Limits */
    public boolean topLimit = false;
    public boolean bottomLimit = false;

    /* Current position of the elevator motors (Height of the Elevator) */
    public Distance leftHeight = Meters.zero();
    public Distance rightHeight = Meters.zero();

    /* Current rotation of the elevator motors */
    public Angle leftMotorAngle = Rotations.zero();
    public Angle rightMotorAngle = Rotations.zero();

    /* Current velocity of the elevator motors */
    public LinearVelocity leftVelocity = MetersPerSecond.zero();
    public LinearVelocity rightVelocity = MetersPerSecond.zero();

    /* Current acceleration of the elevator motors */
    public LinearAcceleration leftAcceleration = MetersPerSecondPerSecond.zero();
    public LinearAcceleration rightAcceleration = MetersPerSecondPerSecond.zero();

    /* Applied voltage to the elevator motors */
    public Voltage leftAppliedVolts = Volts.zero();
    public Voltage rightAppliedVolts = Volts.zero();

    /* Current draw of the elevator motors */
    public Current leftCurrentAmps = Amps.zero();
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
   * Sets the target position for the elevator using closed-loop control and motion magic.
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

  /**
   * Resets both motors' (left and right) dead-reckoning position to the rotor count consistent with
   * the input height
   *
   * @param height The height that the elevator should reference as zero
   */
  public default void resetRotorPositions(Distance height) {}

  /** Stops all motors */
  public default void stop() {}
}
