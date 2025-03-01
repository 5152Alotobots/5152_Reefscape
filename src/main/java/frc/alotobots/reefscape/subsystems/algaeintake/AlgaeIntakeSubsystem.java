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
package frc.alotobots.reefscape.subsystems.algaeintake;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeConstants.Limits.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.algaeintake.io.AlgaeIntakeIO;
import frc.alotobots.reefscape.subsystems.algaeintake.io.AlgaeIntakeIOInputsAutoLogged;
import frc.alotobots.reefscape.util.ControlType;
import org.littletonrobotics.junction.Logger;

/**
 * The AlgaeIntake subsystem controls the robot's intake mechanism for game pieces using two Neo 550
 * motors. This subsystem manages both closed-loop velocity control and open loop control modes.
 * Positive motor output pulls inward (intake direction), negative output pushes outward. The two
 * motors spin in opposite directions to properly intake/outtake game pieces.
 */
public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** The hardware interface for controlling and monitoring the intake mechanism */
  private final AlgaeIntakeIO io;

  /** Auto-logged inputs from intake sensors and motor controllers */
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  /**
   * Constructs a new AlgaeIntakeSubsystem with the specified hardware interface.
   *
   * @param io The hardware interface for controlling the intake mechanism
   */
  public AlgaeIntakeSubsystem(AlgaeIntakeIO io) {
    this.io = io;
  }

  /**
   * Updates and logs intake sensor inputs. Called periodically by the command scheduler. This
   * method ensures that the latest sensor data is available for control decisions.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake", inputs);
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

    io.setAlgaeIntakeOpenLoop(adjustedOutput);
    Logger.recordOutput("AlgaeIntake/ControlType", ControlType.OpenLoop.OPEN_LOOP);
  }

  /**
   * Controls the elevator to move to a specified velocity using closed-loop velocity control.
   *
   * @param velocity Target velocity in meters per second, automatically constrained between
   *     -MAX_OPERATOR_VELOCITY and MAX_OPERATOR_VELOCITY
   */
  public void runToTargetVelocity(AngularVelocity velocity) {
    AngularVelocity adjustedVelocity =
        DegreesPerSecond.of(
            MathUtil.clamp(
                velocity.in(DegreesPerSecond),
                -MAX_VELOCITY.in(DegreesPerSecond),
                MAX_VELOCITY.in(DegreesPerSecond)));
    io.setAlgaeIntakeVelocity(adjustedVelocity, ControlType.ClosedLoop.VELOCITY.ordinal());
    Logger.recordOutput("Elevator/ControlType", ControlType.ClosedLoop.VELOCITY);
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
    return inputs.canRangeInProximity;
  }
}
