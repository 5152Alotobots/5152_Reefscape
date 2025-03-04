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
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface defining the hardware abstraction layer for the wrist subsystem. This interface allows
 * for different hardware implementations (real, simulated, etc.) while maintaining consistent
 * functionality.
 */
public interface WristIO {
  /** Data structure for inputs from wrist hardware. */
  @AutoLog
  public static class WristIOInputs {
    /** Current PID slot being used (0 for velocity, 1 for position) */
    public int pidSlot = 0;

    /** Whether the motor controller is connected */
    public boolean motorConnected = false;

    /** Whether the CANCoder is connected */
    public boolean cancoderConnected = false;

    /** Whether the top limit switch/soft limit is triggered */
    public boolean topLimit = false;

    /** Whether the bottom limit switch/soft limit is triggered */
    public boolean bottomLimit = false;
    public Angle bottomLimitAngle = Rotations.zero();

    /** Current angle of the wrist mechanism */
    public Angle mechanismAngle = Rotations.zero();
    public Distance mechanismHeight = Meters.zero();

    /** Current angular velocity of the wrist */
    public AngularVelocity rotationVelocity = RotationsPerSecond.zero();

    public AngularAcceleration rotationAcceleration = RotationsPerSecondPerSecond.zero();

    /** Current voltage being applied to the motor */
    public Voltage motorAppliedVolts = Volts.zero();

    /** Current being drawn by the motor */
    public Current motorCurrent = Amps.zero();
  }

  /**
   * Updates the wrist input values from hardware.
   *
   * @param inputs The input object to update with the latest hardware state
   */
  public default void updateInputs(WristIOInputs inputs) {}

  public default void setBottomLimitAngle(Angle bottomLimitAngle) {}

  /**
   * Sets the wrist to run to a target position using closed-loop control.
   *
   * @param rotation The target angle to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  public default void setWristPosition(
      Angle rotation, int pidSlot) {}

  /**
   * Sets the wrist to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  public default void setWristVelocity(
      AngularVelocity velocity, int pidSlot) {}

  /**
   * Sets the wrist to run at a target position using motion magic control.
   *
   * @param position The target position to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  public default void setWristPositionMotionMagic(
      Angle position, int pidSlot) {}

  /**
   * Runs the wrist using direct percentage output (open-loop control).
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  public default void setWristOpenLoop(double percentOutput) {}

  /** Stops all wrist motor movement. */
  public default void stop() {}
}
