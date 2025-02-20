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
package frc.alotobots.reefscape.subsystems.coralIntake;

import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Limits.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.coralIntake.io.CoralIntakeIO;
import frc.alotobots.reefscape.subsystems.coralIntake.io.CoralIntakeIOInputsAutoLogged;
import frc.alotobots.reefscape.util.ControlType;
import org.littletonrobotics.junction.Logger;

/**
 * The CoralIntake subsystem controls the robot's intake mechanism for game pieces. This subsystem
 * manages both closed-loop velocity control and open loop control modes. Positive motor output
 * pulls inward (intake direction), negative output pushes outward.
 */
public class CoralIntakeSubsystem extends SubsystemBase {
  /** The hardware interface for controlling and monitoring the intake mechanism */
  private final CoralIntakeIO io;

  /** Auto-logged inputs from intake sensors and motor controllers */
  private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

  /**
   * Constructs a new CoralIntakeSubsystem with the specified hardware interface.
   *
   * @param io The hardware interface for controlling the intake mechanism
   */
  public CoralIntakeSubsystem(CoralIntakeIO io) {
    this.io = io;
  }

  /**
   * Updates and logs intake sensor inputs. Called periodically by the command scheduler. This
   * method ensures that the latest sensor data is available for control decisions.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);
  }

  /**
   * Controls the intake to move at a specified velocity using closed-loop velocity control. [NOT
   * YET IMPLEMENTED]
   *
   * @param velocity Target velocity in radians per second. Positive values indicate intake
   *     direction.
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    // TODO: Implement closed-loop velocity control
    Logger.recordOutput("CoralIntake/ControlType", ControlType.ClosedLoop.VELOCITY);
  }

  /**
   * Controls the intake using direct percent output (open-loop control). Positive values pull
   * inward (intake), negative values push outward.
   *
   * @param percentOutput Motor output percentage (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    double adjustedOutput =
        MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);

    io.setIntakeOpenLoop(adjustedOutput);
    Logger.recordOutput("CoralIntake/ControlType", ControlType.OpenLoop.OPEN_LOOP);
  }

  /**
   * Stops intake movement. This method should be called when active control of the intake is no
   * longer needed.
   */
  public void stop() {
    io.stop();
  }

  /**
   * Checks if the intake currently has a game piece.
   *
   * @return true if a game piece is detected in the intake
   */
  public boolean isIntakeOccupied() {
    return inputs.intakeOccupied;
  }

  /**
   * Gets the current velocity of the intake. Positive values indicate intake direction.
   *
   * @return The current angular velocity
   */
  public AngularVelocity getCurrentVelocity() {
    return inputs.motorVelocity;
  }
}
