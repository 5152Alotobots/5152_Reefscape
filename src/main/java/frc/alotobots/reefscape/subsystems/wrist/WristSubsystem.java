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

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.MAX_ANGLE;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.MAX_OPEN_LOOP_PERCENTAGE;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.MAX_SPEED;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.MIN_ANGLE;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Thresholds.AT_TARGET_ANGLE_POSITION_THRESHOLD;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Thresholds.AT_TARGET_ANGLE_TIME_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXRealConstants.AlgaeMotionProfilingConstants;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIO;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOInputsAutoLogged;
import frc.alotobots.reefscape.util.ControlType;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's wrist mechanism. Handles both position and velocity control
 * of the wrist joint with dynamic limits based on elevator height zones.
 */
public class WristSubsystem extends SubsystemBase {
  /** Hardware abstraction for the wrist */
  private final WristIO io;

  /** Latest inputs from the wrist hardware */
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Timer profileTimer = new Timer();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetAngleDebounce =
      new Debouncer(AT_TARGET_ANGLE_TIME_THRESHOLD.in(Seconds));

  /**
   * Angle object that tracks the currently selected position (maintains last position if not in
   * POSITION control mode)
   */
  private Angle targetAngle = Degrees.zero();

  private final TrapezoidProfile algaeMotionProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              AlgaeMotionProfilingConstants.CRUISE_VELOCITY.in(RotationsPerSecond),
              AlgaeMotionProfilingConstants.ACCELERATION.in(RotationsPerSecondPerSecond)));

  /**
   * Creates a new WristSubsystem.
   *
   * @param io The hardware abstraction interface for the wrist
   * @param elevatorHeightSupplier Supplier function that provides the current elevator height
   */
  public WristSubsystem(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update hardware inputs
    io.updateInputs(inputs);
    Logger.recordOutput("Wrist/TargetAngle", targetAngle.in(Degree));
    Logger.processInputs("Wrist", inputs);
  }

  /**
   * Commands the wrist to move to a target angle using closed-loop control. Target angle is
   * dynamically clamped based on current elevator height zone.
   *
   * @param angle The target angle for the wrist
   */
  public void runToTargetAngle(Angle angle) {
    var adjustedAngle =
        Degrees.of(MathUtil.clamp(angle.in(Degrees), MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees)));
    targetAngle = adjustedAngle;
    io.setWristPositionMotionMagic(adjustedAngle, 1);

    Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.POSITION_MAGIC);
  }

  public Command wristRunToAngleManualProfile(Angle angle) {
    var adjustedAngle =
        Degrees.of(MathUtil.clamp(angle.in(Degrees), MIN_ANGLE.in(Degrees), MAX_ANGLE.in(Degrees)));

    TrapezoidProfile.State goal = new TrapezoidProfile.State(adjustedAngle.in(Rotation), 0);
    TrapezoidProfile.State current =
        new TrapezoidProfile.State(
            inputs.mechanismAngle.in(Rotation), inputs.rotationVelocity.in(RotationsPerSecond));

    return startRun(
            () -> {
              profileTimer.restart();
              targetAngle = adjustedAngle;
            },
            () -> {
              Logger.recordOutput("Wrist/ProfileSetAngle", adjustedAngle.in(Rotations));
              Logger.recordOutput("Wrist/Timer", profileTimer.get());
              Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.POSITION_PROFILED);
              Logger.recordOutput(
                  "Wrist/ProfileFinished", algaeMotionProfile.isFinished(profileTimer.get()));
              Logger.recordOutput("Wrist/ProfileGoalPos", goal.position);
              Logger.recordOutput("Wrist/ProfileStartPos", current.position);

              var setpoint = algaeMotionProfile.calculate(profileTimer.get(), current, goal);
              Logger.recordOutput("Wrist/ProfileVelocity", setpoint.velocity);
              Logger.recordOutput("Wrist/ProfilePosition", setpoint.position);

              io.setWristPosition(setpoint.position, setpoint.velocity, 1);
            })
        .until(this::isAtTargetAngle);
  }

  /**
   * Controls the wrist to move to a specified velocity using closed-loop velocity control. Dynamic
   * limits based on current elevator height are passed to the IO layer.
   *
   * @param velocity Target velocity in degrees per second
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    // Clamp velocity magnitude
    AngularVelocity adjustedVelocity =
        DegreesPerSecond.of(
            MathUtil.clamp(
                velocity.in(DegreesPerSecond),
                -MAX_SPEED.in(DegreesPerSecond),
                MAX_SPEED.in(DegreesPerSecond)));

    io.setWristVelocity(adjustedVelocity, 0);

    Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.VELOCITY);
  }

  /**
   * Runs the wrist using direct percent output (open-loop control). Dynamic limits based on current
   * elevator height are passed to the IO layer.
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    // Clamp percent output
    double adjustedSpeed =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);

    // Command the wrist with the adjusted output and dynamic limits
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

    Angle error = targetAngle.minus(inputs.mechanismAngle);

    Logger.recordOutput("Wrist/error", error);

    boolean inSetPointThreshold =
        error.abs(Degree) < AT_TARGET_ANGLE_POSITION_THRESHOLD.in(Degrees);

    Logger.recordOutput("Wrist/inSetPointThreshold", inSetPointThreshold);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetAngleDebounce.calculate(inSetPointThreshold);
  }
}
