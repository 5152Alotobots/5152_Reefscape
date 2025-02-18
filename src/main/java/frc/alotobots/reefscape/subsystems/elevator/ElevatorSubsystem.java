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
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.elevator.constants.ControlType;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIO;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import frc.alotobots.util.Elastic;
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

  /** Timer that handles the checking for at position */
  private final Timer atSetpointTimer = new Timer();

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

    // Set the rotor positions based on CANrange on startup for better reliability
    if (inputs.canrangeConnected && inputs.canrangeDistance.lt(CAN_RANGE_MAX_VALID_DISTANCE)) {
      io.resetRotorPositions(inputs.canrangeDistance);
    } else if (inputs.canrangeConnected) {
      Logger.recordOutput("Elevator/Faults/CANrangeTooFarDuringInit", true);
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Failed to apply offset to elevator",
              "Verify that elevator is at zero position!"));
    }
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
    io.setElevatorPosition(adjustedHeight, ControlType.ClosedLoop.POSITION.ordinal());
    Logger.recordOutput("Elevator/ControlType", ControlType.ClosedLoop.POSITION);
  }

  /**
   * Controls the elevator to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in meters per second, automatically constrained between
   *     -MAX_SPEED and MAX_SPEED
   */
  public void runToTargetVelocity(LinearVelocity velocity) {
    LinearVelocity adjustedVelocity =
        MetersPerSecond.of(
            MathUtil.clamp(
                velocity.in(MetersPerSecond),
                -MAX_SPEED.in(MetersPerSecond),
                MAX_SPEED.in(MetersPerSecond)));
    io.setElevatorVelocity(adjustedVelocity, ControlType.ClosedLoop.VELOCITY.ordinal());
    Logger.recordOutput("Elevator/ControlType", ControlType.ClosedLoop.VELOCITY);
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
   * Checks if the elevator is stably at its target height for a minimum duration. Uses position and
   * time thresholds defined in ElevatorConstants.
   *
   * @return true if the elevator has maintained its target height within tolerance
   */
  public boolean isAtTargetHeight() {
    boolean inSetPointThreshold =
        targetHeight.minus(inputs.leftHeight).abs(Meters)
            < AT_SET_POINT_POSITION_THRESHOLD.in(Meters);
    if (inSetPointThreshold) {
      if (!atSetpointTimer.isRunning()) {
        atSetpointTimer.restart();
      }
      return atSetpointTimer.hasElapsed(AT_SET_POINT_TIME_THRESHOLD.in(Seconds));
    } else {
      atSetpointTimer.stop();
      return false;
    }
  }
}
