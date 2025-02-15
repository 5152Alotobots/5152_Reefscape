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
package frc.alotobots.reefscape.subsystems.elevator.io;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.MAX_HEIGHT;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.MIN_HEIGHT;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Mechanics.PULLEY_CIRCUMFERENCE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOTalonFXSim implements ElevatorIO {
  /** TalonFX-specific PID and motion control constants for coral game piece (Coral mode). */
  private static final class CoralPIDConstants {
    /** Position control proportional gain */
    public static final double KP = 0.1;

    /** Position control integral gain */
    public static final double KI = 0.0;

    /** Position control derivative gain */
    public static final double KD = 0.0;

    /** Acceleration feedforward gain */
    public static final double KA = 0.0;

    /** Gravity compensation gain */
    public static final double KG = 0.0;

    /** Static friction compensation */
    public static final double KS = 0.0;

    /** Velocity feedforward gain */
    public static final double KV = 0.0;
  }

  /** TalonFX-specific PID and motion control constants for algae game piece (Algae mode). */
  private static final class AlgaePIDConstants {
    /** Position control proportional gain */
    public static final double KP = 0.1;

    /** Position control integral gain */
    public static final double KI = 0.0;

    /** Position control derivative gain */
    public static final double KD = 0.0;

    /** Acceleration feedforward gain */
    public static final double KA = 0.0;

    /** Gravity compensation gain */
    public static final double KG = 0.0;

    /** Static friction compensation */
    public static final double KS = 0.0;

    /** Velocity feedforward gain */
    public static final double KV = 0.0;
  }

  /**
   * TalonFX-specific hardware configuration constants defining the mechanical properties of the
   * elevator system.
   */
  private static final class HardwareConfig {
    /** Gear ratio between motor and pulley (motor rotations : pulley rotations) */
    static final double GEAR_RATIO = 10.0;

    /** Weight of full assembly that is being lifted */
    static final Mass MASS = Kilogram.of(5.0);

    /** Conversion factor from TalonFX rotations to meters of travel */
    static final double ROTATIONS_TO_METERS = PULLEY_CIRCUMFERENCE.in(Meters) / GEAR_RATIO;
  }

  // Simulation-specific constants

  private final ElevatorSim elevatorSim;
  private final DCMotor motors = DCMotor.getKrakenX60Foc(2);

  // State tracking
  private double appliedVolts = 0.0;
  private int currentPidSlot = 0;
  private boolean brakeMode = true;

  public ElevatorIOTalonFXSim() {
    elevatorSim =
        new ElevatorSim(
            motors,
            HardwareConfig.GEAR_RATIO, // Using from your hardware config
            HardwareConfig.MASS.in(Kilograms),
            PULLEY_CIRCUMFERENCE.in(Meters) / (2 * Math.PI), // Convert circumference to radius
            MIN_HEIGHT.in(Meters),
            MAX_HEIGHT.in(Meters),
            true,
            0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02); // 50Hz simulation rate

    // Connected status
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;
    inputs.canrangeConnected = true;

    // Limits
    inputs.topLimit = elevatorSim.getPositionMeters() >= MAX_HEIGHT.in(Meters);
    inputs.bottomLimit = elevatorSim.getPositionMeters() <= MIN_HEIGHT.in(Meters);

    // Position (both motors and CANRange report same position in sim)
    Distance position = Meters.of(elevatorSim.getPositionMeters());
    inputs.leftHeight = position;
    inputs.rightHeight = position;
    inputs.canrangeDistance = position;

    // Velocity
    LinearVelocity velocity = MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
    inputs.leftVelocity = velocity;
    inputs.rightVelocity = velocity;

    // Voltage
    inputs.leftAppliedVolts = Volts.of(appliedVolts);
    inputs.rightAppliedVolts = Volts.of(appliedVolts);

    // Current (split total current between motors)
    double totalCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.leftCurrentAmps = Amps.of(totalCurrent / 2.0);
    inputs.rightCurrentAmps = Amps.of(totalCurrent / 2.0);
  }

  @Override
  public void setElevatorPosition(Distance position, int pidSlot) {
    currentPidSlot = pidSlot;
    // Simulate position control with basic P controller
    double error = position.in(Meters) - elevatorSim.getPositionMeters();
    double kP = (pidSlot == 0) ? CoralPIDConstants.KP : AlgaePIDConstants.KP;
    appliedVolts = error * kP;
    appliedVolts = Math.min(12.0, Math.max(-12.0, appliedVolts));
    elevatorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setElevatorOpenLoop(double percentOutput) {
    appliedVolts = percentOutput * 12.0;
    elevatorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setElevatorBrakeMode(boolean brake) {
    brakeMode = brake;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    elevatorSim.setInputVoltage(0.0);
  }
}
