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
package frc.alotobots.reefscape.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Thresholds.AT_TARGET_HEIGHT_POSITION_THRESHOLD;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Thresholds.AT_TARGET_HEIGHT_TIME_THRESHOLD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIO;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import frc.alotobots.reefscape.util.ControlType;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls the vertical movement of the robot's elevator mechanism. This
 * subsystem manages closed-loop position control, closed-loop velocity control, and open loop
 * control
 */
public class ElevatorSubsystem extends SubsystemBase {
  /** The hardware interface for controlling and monitoring the elevator mechanism. */
  private final ElevatorIO io;

  /** Auto-logged inputs from elevator sensors and motor controllers. */
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetHeightDebounce =
      new Debouncer(AT_TARGET_HEIGHT_TIME_THRESHOLD.in(Seconds));

  /** Debouncer for bottom resetting logic */
  private final Debouncer atBottomDebounce = new Debouncer(AT_TARGET_HEIGHT_TIME_THRESHOLD.in(Seconds));

  /** Boolean tracking if the elevator has reset from the CANrange in its current position */
  private boolean hasReset = false;

  /**
   * Distance object that tracks the currently selected position (maintains last position if not in
   * POSITION control mode)
   */
  private Distance targetHeight = Meters.zero();

  /**
   * Constructs a new ElevatorSubsystem with the specified hardware interface.
   *
   * @param io The hardware interface for controlling the elevator mechanism
   */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  /**
   * Updates and logs elevator sensor inputs. Called periodically by the command scheduler. This
   * method ensures that the latest sensor data is available for control decisions.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/TargetHeight", targetHeight);

    // Automatically check every loop to see if the sensor should reset the position of the elevator
    handleBottomReset();
  }

  /**
   * Controls the elevator to move to a specified height using closed-loop position control.
   *
   * @param height Target height in meters, automatically constrained between MIN_HEIGHT and
   *     MAX_HEIGHT
   */
  public void runToTargetPosition(Distance height) {
    Distance adjustedHeight =
        Meters.of(MathUtil.clamp(height.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters)));
    targetHeight = adjustedHeight;
    io.setElevatorPositionMotionMagic(adjustedHeight, ControlType.ClosedLoop.POSITION.ordinal());
    Logger.recordOutput("Elevator/ControlType", ControlType.ClosedLoop.POSITION);
  }

  /**
   * Controls the elevator to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in meters per second, automatically constrained between
   *     -MAX_OPERATOR_VELOCITY and MAX_OPERATOR_VELOCITY
   */
  public void runToTargetVelocity(LinearVelocity velocity) {
    LinearVelocity adjustedVelocity = applyVelocityLimitIfNeeded(velocity);
    io.setElevatorVelocity(adjustedVelocity, ControlType.ClosedLoop.VELOCITY.ordinal());
    Logger.recordOutput("Elevator/ControlType", ControlType.ClosedLoop.VELOCITY);
  }

  /**
   * Controls the elevator to move to a specified velocity using closed-loop climbing velocity
   * control.
   *
   * @param velocity Target velocity in meters per second, automatically constrained between
   *     -MAX_OPERATOR_VELOCITY and MAX_OPERATOR_VELOCITY
   */
  public void runToClimbingVelocity(LinearVelocity velocity) {
    LinearVelocity adjustedVelocity =
        MetersPerSecond.of(
            MathUtil.clamp(
                velocity.in(MetersPerSecond),
                -MAX_OPERATOR_VELOCITY.in(MetersPerSecond),
                MAX_OPERATOR_VELOCITY.in(MetersPerSecond)));
    io.setElevatorVelocity(adjustedVelocity, ControlType.ClosedLoop.VELOCITY_CLIMB.ordinal());
    Logger.recordOutput("Elevator/ControlType", ControlType.ClosedLoop.VELOCITY_CLIMB);
  }

  /**
   * Controls the elevator using direct percent output (open-loop control). The output is clamped to
   * prevent excessive speed.
   *
   * @param percentOutput Motor output percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    double adjustedSpeed =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);
    io.setElevatorOpenLoop(adjustedSpeed);
    Logger.recordOutput("Elevator/ControlType", ControlType.OpenLoop.OPEN_LOOP);
  }

  /**
   * Stops elevator movement and enables brake mode to maintain position. This method should be
   * called when active control of the elevator is no longer needed.
   */
  public void stop() {
    io.stop();
    io.setElevatorBrakeMode(true);
  }

  /**
   * Retrieves the current height of the elevator.
   *
   * @return The current height as a Distance object
   */
  public Distance getCurrentHeight() {
    return inputs.leftHeight;
  }

  /**
   * Checks if the elevator is stably at its target height for a minimum duration.
   *
   * @return true if the elevator has maintained its target height within tolerance
   */
  public boolean isAtTargetHeight() {
    // Check if current height is within threshold of target
    boolean inSetPointThreshold =
        targetHeight.minus(inputs.leftHeight).abs(Meters)
            < AT_TARGET_HEIGHT_POSITION_THRESHOLD.in(Meters);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetHeightDebounce.calculate(inSetPointThreshold);
  }

  /**
   * Handles resetting the elevator's position when it reaches the bottom. Uses a debounced bottom
   * detection to ensure the elevator is stable before resetting. Will only reset once per bottom
   * detection to prevent multiple resets. The position will not reset again until the elevator
   * moves away from the bottom and returns.
   */
  private void handleBottomReset() {
    if (atBottomDebounce.calculate(inputs.canrangeInProximity) && !hasReset) {
      io.resetRotorPositions(MIN_HEIGHT);
      hasReset = true;
      Logger.recordOutput("Elevator/PositionReset", true);
    } else if (!inputs.canrangeInProximity) {
      hasReset = false;
      Logger.recordOutput("Elevator/PositionReset", false);
    }
  }

  /**
   * Checks if the elevator is within the velocity limit distance from the top or bottom limits.
   *
   * @return true if the elevator is within the velocity limit distance, false otherwise
   */
  private boolean isVelocityLimitNeeded() {
    return inputs.leftHeight.minus(MIN_HEIGHT).abs(Meters) < DISTANCE_FROM_LIMIT.in(Meters)
        || inputs.leftHeight.minus(MAX_HEIGHT).abs(Meters) < DISTANCE_FROM_LIMIT.in(Meters);
  }

  /**
   * Applies the velocity limit if the elevator is within the velocity limit distance from the top
   * or bottom limits.
   *
   * @param velocity The target velocity
   * @return The adjusted velocity if the limit is needed, otherwise the original velocity
   */
  private LinearVelocity applyVelocityLimitIfNeeded(LinearVelocity velocity) {
    if (isVelocityLimitNeeded()) {
      return MetersPerSecond.of(
          MathUtil.clamp(
              velocity.in(MetersPerSecond),
              -MAX_VELOCITY_NEAR_LIMIT.in(MetersPerSecond),
              MAX_VELOCITY_NEAR_LIMIT.in(MetersPerSecond)));
    } else {
      return MetersPerSecond.of(
          MathUtil.clamp(
              velocity.in(MetersPerSecond),
              -MAX_OPERATOR_VELOCITY.in(MetersPerSecond),
              MAX_OPERATOR_VELOCITY.in(MetersPerSecond)));
    }
  }
}
