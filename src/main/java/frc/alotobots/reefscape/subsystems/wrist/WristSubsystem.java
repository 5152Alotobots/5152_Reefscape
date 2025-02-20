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
package frc.alotobots.reefscape.subsystems.wrist;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Thresholds.AT_SET_POINT_POSITION_THRESHOLD;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Thresholds.AT_SET_POINT_TIME_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIO;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOInputsAutoLogged;
import frc.alotobots.reefscape.util.ControlType;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's wrist mechanism. Handles both position and velocity control
 * of the wrist joint.
 */
public class WristSubsystem extends SubsystemBase {
  /** Hardware abstraction for the wrist */
  private WristIO io;

  /** Latest inputs from the wrist hardware */
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Timer that handles the checking for at position */
  private final Timer atSetpointTimer = new Timer();

  /**
   * Angle object that tracks the currently selected position (maintains last position if not in
   * POSITION control mode)
   */
  private Angle targetAngle = Degrees.zero();

  /**
   * Creates a new WristSubsystem.
   *
   * @param io The hardware abstraction interface for the wrist
   */
  public WristSubsystem(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  /**
   * Commands the wrist to move to a target angle using closed-loop control. Target angle is clamped
   * within the allowed range.
   *
   * @param angle The target angle for the wrist
   */
  public void runToTargetAngle(Angle angle) {
    Angle adjustedAngle =
        Degrees.of(MathUtil.clamp(angle.in(Degrees), MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees)));
    targetAngle = adjustedAngle;
    io.setWristPosition(adjustedAngle, ControlType.ClosedLoop.POSITION.ordinal());
    Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.POSITION);
  }

  /**
   * Controls the wrist to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in degrees per second, automatically constrained between
   *     -MAX_SPEED and MAX_SPEED
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        DegreesPerSecond.of(
            MathUtil.clamp(
                velocity.in(DegreesPerSecond),
                -MAX_SPEED.in(DegreesPerSecond),
                MAX_SPEED.in(DegreesPerSecond)));
    io.setWristVelocity(adjustedVelocity, ControlType.ClosedLoop.VELOCITY.ordinal());
    Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.VELOCITY);
  }

  /**
   * Runs the wrist using direct percent output (open-loop control). Output is clamped within the
   * allowed range.
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    double adjustedSpeed =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);
    io.setWristOpenLoop(adjustedSpeed);
    Logger.recordOutput("Wrist/ControlType", ControlType.OpenLoop.OPEN_LOOP);
  }

  /** Stops all wrist movement. */
  public void stop() {
    io.stop();
  }

  /**
   * Retrieves the current angle of the wrist.
   *
   * @return The current angle as an Angle object
   */
  public Angle getCurrentAngle() {
    return inputs.mechanismAngle;
  }

  /**
   * Checks if the wrist is stably at its target angle for a minimum duration.
   *
   * @return true if the wrist has maintained its target angle within tolerance
   */
  public boolean isAtTargetAngle() {
    // Check if current angle is within threshold of target
    boolean inSetPointThreshold =
            targetAngle.minus(inputs.mechanismAngle).abs(Degrees)
                    < AT_SET_POINT_POSITION_THRESHOLD.in(Degrees);

    // Only start if in position threshold
    if (inSetPointThreshold) {
      // Start timer if not running and check elapsed time
      if (!atSetpointTimer.isRunning()) {
        atSetpointTimer.restart();
      }
      // Return true if wrist has been at position for minimum duration
      return atSetpointTimer.hasElapsed(AT_SET_POINT_TIME_THRESHOLD.in(Seconds));
    } else {
      // Reset timer if outside threshold
      atSetpointTimer.stop();
      return false;
    }
  }
}
