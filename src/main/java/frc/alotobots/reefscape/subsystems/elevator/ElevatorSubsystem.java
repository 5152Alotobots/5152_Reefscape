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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.AT_SET_POINT_THRESHOLD;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.MAX_OPEN_LOOP_PERCENTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.elevator.constants.ControlType;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIO;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
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
    io.setElevatorPosition(adjustedHeight, ControlType.POSITION.ordinal());
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
    io.setElevatorVelocity(adjustedVelocity, ControlType.VELOCITY.ordinal());
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
   * Checks if the elevator is within the acceptable range of its target height. Uses the threshold
   * defined in ElevatorConstants.
   *
   * @return true if the elevator is at its target height within tolerance
   */
  public boolean isAtTargetHeight() {
    return inputs.mechanismClosedLoopError.abs(Meters) < AT_SET_POINT_THRESHOLD.in(Meters);
  }
}
