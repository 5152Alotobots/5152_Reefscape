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
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Thresholds.AT_TARGET_ANGLE_POSITION_THRESHOLD;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Thresholds.AT_TARGET_ANGLE_TIME_THRESHOLD;
import static frc.alotobots.reefscape.subsystems.wrist.util.WristLimitZones.DEFAULT_ZONE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIO;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOInputsAutoLogged;
import frc.alotobots.reefscape.subsystems.wrist.util.WristLimitZones;
import frc.alotobots.reefscape.util.ControlType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling the robot's wrist mechanism. Handles both position and velocity control
 * of the wrist joint with dynamic limits based on elevator height zones.
 */
public class WristSubsystem extends SubsystemBase {
  /** Hardware abstraction for the wrist */
  private final WristIO io;

  /** Supplier for the current elevator height */
  private final Supplier<Distance> elevatorHeightSupplier;

  /** Latest inputs from the wrist hardware */
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /** Debouncer for ensuring stability at a position */
  private final Debouncer atTargetAngleDebounce =
      new Debouncer(AT_TARGET_ANGLE_TIME_THRESHOLD.in(Seconds));

  /**
   * Angle object that tracks the currently selected position (maintains last position if not in
   * POSITION control mode)
   */
  private Angle targetAngle = Degrees.zero();

  /** The current limit zone based on elevator height */
  private WristLimitZones.WristLimitZone currentZone = DEFAULT_ZONE;

  /** Flag indicating if automatic recovery is in progress */
  private boolean inAutoRecovery = false;

  /** Target angle for recovery motion */
  private Angle recoveryTargetAngle = Degrees.zero();

  /** Enum to track the type of pending command */
  private enum PendingCommandType {
    NONE,
    POSITION,
    VELOCITY,
    PERCENT_OUTPUT
  }

  /** The type of command pending execution after recovery */
  private PendingCommandType pendingCommandType = PendingCommandType.NONE;

  /** Parameters for pending position command */
  private Angle pendingTargetAngle = Degrees.zero();

  /** Parameters for pending velocity command */
  private AngularVelocity pendingVelocity = DegreesPerSecond.zero();

  /** Parameters for pending percent output command */
  private double pendingPercentOutput = 0.0;

  /**
   * Creates a new WristSubsystem.
   *
   * @param io The hardware abstraction interface for the wrist
   * @param elevatorHeightSupplier Supplier function that provides the current elevator height
   */
  public WristSubsystem(WristIO io, Supplier<Distance> elevatorHeightSupplier) {
    this.io = io;
    this.elevatorHeightSupplier = elevatorHeightSupplier;
  }

  @Override
  public void periodic() {
    // Update hardware inputs
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    // Get current elevator height
    Distance elevatorHeight = elevatorHeightSupplier.get();

    // Store previous zone for change detection
    WristLimitZones.WristLimitZone previousZone = currentZone;

    // Update current zone
    currentZone = WristLimitZones.findZone(elevatorHeight);

    // Check for zone change and if we need to recover
    if (!currentZone.equals(previousZone) && !inAutoRecovery) {
      Angle currentAngle = inputs.mechanismAngle;
      Angle minAngle = currentZone.getMinWristAngle();
      Angle maxAngle = currentZone.getMaxWristAngle();

      // Check if current position is outside new zone limits
      if (currentAngle.lt(minAngle) || currentAngle.gt(maxAngle)) {
        // Start automatic recovery
        startAutoRecovery(currentAngle, minAngle, maxAngle);
      }
    }

    // If in auto recovery, check if complete
    if (inAutoRecovery && isAtTargetAngle()) {
      inAutoRecovery = false;
      Logger.recordOutput("Wrist/AutoRecoveryComplete", true);

      // Execute any pending command
      executePendingCommand();
    }

    // Log current zone information for debugging
    Logger.recordOutput("Wrist/CurrentZoneMin", currentZone.getMinWristAngle().in(Degrees));
    Logger.recordOutput("Wrist/CurrentZoneMax", currentZone.getMaxWristAngle().in(Degrees));
    Logger.recordOutput("Wrist/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Wrist/InAutoRecovery", inAutoRecovery);
  }

  /**
   * Starts automatic recovery motion to bring the wrist within the new zone limits.
   *
   * @param currentAngle Current wrist angle
   * @param minAngle Minimum allowed angle in the new zone
   * @param maxAngle Maximum allowed angle in the new zone
   */
  private void startAutoRecovery(Angle currentAngle, Angle minAngle, Angle maxAngle) {
    // Determine target angle (closest valid position)
    if (currentAngle.lt(minAngle)) {
      recoveryTargetAngle = minAngle;
    } else {
      recoveryTargetAngle = maxAngle;
    }

    // Set recovery flag
    inAutoRecovery = true;
    targetAngle = recoveryTargetAngle;

    // Command wrist to move to recovery target using motion magic
    io.setWristPositionMotionMagic(
        recoveryTargetAngle, ControlType.ClosedLoop.POSITION.ordinal(), minAngle, maxAngle);

    Logger.recordOutput("Wrist/AutoRecoveryTarget", recoveryTargetAngle.in(Degrees));
    Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.POSITION);
  }

  /** Executes any pending command after auto recovery is complete. */
  private void executePendingCommand() {
    switch (pendingCommandType) {
      case POSITION:
        runToTargetAngle(pendingTargetAngle);
        Logger.recordOutput("Wrist/ExecutingQueuedPositionCommand", true);
        break;
      case VELOCITY:
        runToTargetVelocity(pendingVelocity);
        Logger.recordOutput("Wrist/ExecutingQueuedVelocityCommand", true);
        break;
      case PERCENT_OUTPUT:
        runAtPercentOutput(pendingPercentOutput);
        Logger.recordOutput("Wrist/ExecutingQueuedPercentOutputCommand", true);
        break;
      case NONE:
      default:
        // No pending command to execute
        break;
    }

    // Clear pending command
    pendingCommandType = PendingCommandType.NONE;
  }

  /**
   * Commands the wrist to move to a target angle using closed-loop control. Target angle is
   * dynamically clamped based on current elevator height zone.
   *
   * @param angle The target angle for the wrist
   */
  public void runToTargetAngle(Angle angle) {
    // If in auto recovery, store command for later execution
    if (inAutoRecovery) {
      pendingCommandType = PendingCommandType.POSITION;
      pendingTargetAngle = angle;
      Logger.recordOutput("Wrist/CommandQueuedDuringRecovery", true);
      return;
    }

    // Get min and max angle limits from current zone
    Angle minAngle = WristLimitZones.getMinAngle(elevatorHeightSupplier.get());
    Angle maxAngle = WristLimitZones.getMaxAngle(elevatorHeightSupplier.get());

    // Clamp target angle within zone limits
    Angle adjustedAngle =
        Degrees.of(MathUtil.clamp(angle.in(Degrees), minAngle.in(Degrees), maxAngle.in(Degrees)));

    targetAngle = adjustedAngle;

    // Command the wrist to the adjusted angle with dynamic limits
    io.setWristPositionMotionMagic(
        adjustedAngle, ControlType.ClosedLoop.POSITION.ordinal(), minAngle, maxAngle);
    Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.POSITION);
  }

  /**
   * Controls the wrist to move to a specified velocity using closed-loop velocity control. Dynamic
   * limits based on current elevator height are passed to the IO layer.
   *
   * @param velocity Target velocity in degrees per second
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    // If in auto recovery, store command for later execution
    if (inAutoRecovery) {
      pendingCommandType = PendingCommandType.VELOCITY;
      pendingVelocity = velocity;
      Logger.recordOutput("Wrist/CommandQueuedDuringRecovery", true);
      return;
    }

    // Get min and max angle limits from current zone
    Angle minAngle = WristLimitZones.getMinAngle(elevatorHeightSupplier.get());
    Angle maxAngle = WristLimitZones.getMaxAngle(elevatorHeightSupplier.get());

    // Clamp velocity magnitude
    AngularVelocity adjustedVelocity =
        DegreesPerSecond.of(
            MathUtil.clamp(
                velocity.in(DegreesPerSecond),
                -MAX_SPEED.in(DegreesPerSecond),
                MAX_SPEED.in(DegreesPerSecond)));

    // Command the wrist to the adjusted velocity with dynamic limits
    io.setWristVelocity(
        adjustedVelocity, ControlType.ClosedLoop.VELOCITY.ordinal(), minAngle, maxAngle);
    Logger.recordOutput("Wrist/ControlType", ControlType.ClosedLoop.VELOCITY);
  }

  /**
   * Runs the wrist using direct percent output (open-loop control). Dynamic limits based on current
   * elevator height are passed to the IO layer.
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    // If in auto recovery, store command for later execution
    if (inAutoRecovery) {
      pendingCommandType = PendingCommandType.PERCENT_OUTPUT;
      pendingPercentOutput = percentOutput;
      Logger.recordOutput("Wrist/CommandQueuedDuringRecovery", true);
      return;
    }

    // Get min and max angle limits from current zone
    Angle minAngle = WristLimitZones.getMinAngle(elevatorHeightSupplier.get());
    Angle maxAngle = WristLimitZones.getMaxAngle(elevatorHeightSupplier.get());

    // Clamp percent output
    double adjustedSpeed =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);

    // Command the wrist with the adjusted output and dynamic limits
    io.setWristOpenLoop(adjustedSpeed, minAngle, maxAngle);
    Logger.recordOutput("Wrist/ControlType", ControlType.OpenLoop.OPEN_LOOP);
  }

  /** Stops all wrist movement. */
  public void stop() {
    // If in auto recovery, clear pending commands and don't stop
    if (inAutoRecovery) {
      pendingCommandType = PendingCommandType.NONE;
      return;
    }

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
            < AT_TARGET_ANGLE_POSITION_THRESHOLD.in(Degrees);

    // Use debouncer to check if we've been at setpoint for the required duration
    return atTargetAngleDebounce.calculate(inSetPointThreshold);
  }
}
