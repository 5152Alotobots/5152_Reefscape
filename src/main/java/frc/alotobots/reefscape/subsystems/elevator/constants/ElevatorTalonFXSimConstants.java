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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.experimental.UtilityClass;

/**
 * Constants for the physical elevator subsystem using TalonFX motors. Contains PID constants for
 * different control modes and motor safety limits.
 */
@UtilityClass
public final class ElevatorTalonFXSimConstants {

  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {
    /** TalonFX-specific PID and motion control constants for velocity (Velocity mode). */
    public static final class VelocityPIDConstants {
      /** Position control proportional gain */
      public static final double KP = 0.1;

      /** Position control integral gain */
      public static final double KI = 0.0;

      /** Position control derivative gain */
      public static final double KD = 0.0;

      /** Acceleration feedforward gain */
      public static final double KA = 0.0;

      /** Gravity compensation gain */
      public static final double KG = 0.16;

      /** Static friction compensation */
      public static final double KS = 0.0;

      /** Velocity feedforward gain */
      public static final double KV = 0.11;
    }

    /** TalonFX-specific PID and motion control constants for Position mode (Position mode). */
    public static final class PositionPIDConstants {
      /** Position control proportional gain */
      public static final double KP = 3.3;

      /** Position control integral gain */
      public static final double KI = 0.0;

      /** Position control derivative gain */
      public static final double KD = 0.0;

      /** Acceleration feedforward gain */
      public static final double KA = 0.0;

      /** Gravity compensation gain */
      public static final double KG = 0.16;

      /** Static friction compensation */
      public static final double KS = 0.19;

      /** Velocity feedforward gain */
      public static final double KV = 0.0;
    }

    /** TalonFX-specific PID and motion control constants for climbing mode. */
    public static final class ClimbingPIDConstants {
      /** Climbing control proportional gain */
      public static final double KP = 0.2;

      /** Climbing control integral gain */
      public static final double KI = 0.0;

      /** Climbing control derivative gain */
      public static final double KD = 0.0;

      /** Acceleration feedforward gain */
      public static final double KA = 0.0;

      /** Gravity compensation gain */
      public static final double KG = 0.16;

      /** Static friction compensation */
      public static final double KS = 0.0;

      /** Velocity feedforward gain */
      public static final double KV = 0.12;
    }
  }

  public static final class MotionMagicConstants {
    public static final LinearVelocity CRUISE_VELOCITY = MetersPerSecond.of(1.65);
    public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(2.9);
    public static final double JERK = 0;
  }

  public static final class HardwareConstants {
    public static final double GEAR_RATIO = 15.0;
    public static final double PULLEY_RADIUS_M = 0.04;
    public static final double ELEVATOR_MASS_KG = 5;
  }

  /** Contains safety limit constants for the elevator motors. */
  public static final class MotorSafetyLimits {

    /** Maximum stator current limit in amperes */
    public static final Current STATOR_AMP_LIMIT = Amps.of(35);
  }

  /** Direction of the left motor rotation */
  public static final InvertedValue LEFT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final NeutralModeValue MECHANISM_NEUTRAL_MODE = NeutralModeValue.Brake;

  /** Regression used to calculate height of motor. (Should be linear) Rotations:Meters */
  public static final double HEIGHT_PER_ROTATION = 0.00977762;
}
