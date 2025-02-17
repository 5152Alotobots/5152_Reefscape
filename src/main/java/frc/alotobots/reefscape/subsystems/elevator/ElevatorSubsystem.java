/*
 * ALOTOBOTS - FRC Team 5152
 * https://github.com/5152Alotobots
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
 * The Elevator subsystem controls the vertical movement of the robot's elevator mechanism.
 * It supports different PID configurations based on the game element being handled and
 * provides both position-based and manual control options.
 */
public class ElevatorSubsystem extends SubsystemBase {
  /** The hardware interface for the elevator. */
  private final ElevatorIO io;

  /** Logged inputs from the elevator hardware. */
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Supplier that provides information about the current game element in the intake. */
  private final Supplier<GameElement> elementInIntake;

  /**
   * Creates a new ElevatorSubsystem.
   *
   * @param io The hardware interface for the elevator
   * @param elementInIntake Supplier that provides the current game element in the intake
   */
  public ElevatorSubsystem(ElevatorIO io, Supplier<GameElement> elementInIntake) {
    this.io = io;
    this.elementInIntake = elementInIntake;
  }

  /**
   * Periodic update function that updates and logs elevator inputs.
   * This method is called periodically by the command scheduler.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /**
   * Runs the elevator to a target position with PID control, using different PID slots
   * based on the current game element.
   *
   * @param height The target height in meters, automatically clamped between MIN_HEIGHT and MAX_HEIGHT
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
   * Runs the elevator using direct percent output control.
   *
   * @param percentOutput The percent output to apply to the elevator motor (-1.0 to 1.0)
   */
  public void runAtPercentOutput(double percentOutput) {
    double adjustedSpeed = MathUtil.clamp(percentOutput, -MAX_OPEN_LOOP_PERCENTAGE, MAX_OPEN_LOOP_PERCENTAGE);
    io.setElevatorOpenLoop(adjustedSpeed);
  }

  /**
   * Stops the elevator movement and enables brake mode.
   * This method should be called when the elevator needs to maintain its position.
   */
  public void stop() {
    io.stop();
    io.setElevatorBrakeMode(true);
  }

  /** Getter method that returns the current lift height
   * @return Distance representing the height of the lift
   * */
  public Distance getCurrentHeight() {
    return inputs.leftHeight;
  }

  /** Checks to see if the lift is within the acceptable range specified in {@link ElevatorConstants}
   * @return True if within range
   * */
  public boolean isAtTargetHeight() {
    return inputs.mechanismClosedLoopError.abs(Meters) < AT_SET_POINT_THRESHOLD.in(Meters);
  }
}