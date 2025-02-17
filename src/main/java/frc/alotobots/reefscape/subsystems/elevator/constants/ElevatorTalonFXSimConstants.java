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
package frc.alotobots.reefscape.subsystems.elevator.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/**
 * Constants for the simulated elevator subsystem using TalonFX motors. Contains PID constants for
 * different game piece modes and physical configuration parameters.
 */
public class ElevatorTalonFXSimConstants {
  /** TalonFX-specific PID and motion control constants for no game piece (Empty mode). */
  public static final class EmptyPIDConstants {
    /** Position control proportional gain */
    public static final double KP = 3.3;

    /** Position control integral gain */
    public static final double KI = 0.0;

    /** Position control derivative gain */
    public static final double KD = 0.0;

    /** Acceleration feedforward gain */
    public static final double KA = 0.0;

    /** Gravity compensation gain */
    public static final double KG = 0.13;

    /** Static friction compensation */
    public static final double KS = 0.19;

    /** Velocity feedforward gain */
    public static final double KV = 0.0;
  }
  /** TalonFX-specific PID and motion control constants for coral/algae game piece (Coral/Algae mode). */
  public static final class CoralAlgaePIDConstants {
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

  /** TalonFX-specific PID and motion control constants for cage game piece (Cage mode). */
  public static final class CagePIDConstants {
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
  public static final class HardwareConfig {
    /** Gear ratio between motor and pulley (motor rotations : pulley rotations) */
    public static final double GEAR_RATIO = 35.0;

    /** Weight of full assembly that is being lifted */
    public static final Mass MASS = Pounds.of(5.2);
  }

  /** Physical dimensions and mechanical configuration of the elevator system. */
  public static final class Mechanics {
    /** Diameter of the elevator pulley */
    public static final Distance PULLEY_DIAMETER = Centimeters.of(3.2);

    /** Circumference of the elevator pulley */
    public static final Distance PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);
  }
}
