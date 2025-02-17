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
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.AT_SET_POINT_THRESHOLD;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.MAX_HEIGHT;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.MIN_HEIGHT;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.MAX_OPEN_LOOP_PERCENTAGE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIO;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOInputsAutoLogged;
import frc.alotobots.reefscape.util.GameElement;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls the vertical movement of the robot's elevator mechanism. This
 * subsystem manages both closed-loop position control and open-loop manual control of the elevator,
 * with different PID configurations based on the game element being handled.
 */
public class ElevatorSubsystem extends SubsystemBase {
  /** The hardware interface for controlling and monitoring the elevator mechanism. */
  private final ElevatorIO io;

  /** Auto-logged inputs from elevator sensors and motor controllers. */
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Provides real-time information about the current game element in the intake. */
  private final Supplier<GameElement> elementInIntake;

  /**
   * Constructs a new ElevatorSubsystem with the specified hardware interface and game element
   * supplier.
   *
   * @param io The hardware interface for controlling the elevator mechanism
   * @param elementInIntake Supplier that provides information about the current game element
   */
  public ElevatorSubsystem(ElevatorIO io, Supplier<GameElement> elementInIntake) {
    this.io = io;
    this.elementInIntake = elementInIntake;
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
   * Controls the elevator to move to a specified height using closed-loop position control. The PID
   * configuration is automatically selected based on the current game element.
   *
   * @param height Target height in meters, automatically constrained between MIN_HEIGHT and
   *     MAX_HEIGHT
   */
  public void runToTargetPosition(Distance height) {
    Distance adjustedHeight =
        Meters.of(MathUtil.clamp(height.in(Meters), MIN_HEIGHT.in(Meters), MAX_HEIGHT.in(Meters)));
    switch (elementInIntake.get()) {
      case NONE:
        io.setElevatorPosition(adjustedHeight, ElevatorConstants.PIDSlot.NONE);
        break;
      case CORAL:
        io.setElevatorPosition(adjustedHeight, ElevatorConstants.PIDSlot.CORAL);
        break;
      case ALGAE:
        io.setElevatorPosition(adjustedHeight, ElevatorConstants.PIDSlot.ALGAE);
        break;
    }
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
