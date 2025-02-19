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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIO;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOInputsAutoLogged;
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
    io.setWristPosition(adjustedAngle, 1);
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
  }

  /** Stops all wrist movement. */
  public void stop() {
    io.stop();
  }
}
