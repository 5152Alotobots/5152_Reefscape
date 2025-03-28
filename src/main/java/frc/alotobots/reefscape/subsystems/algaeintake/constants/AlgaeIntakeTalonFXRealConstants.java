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
package frc.alotobots.reefscape.subsystems.algaeintake.constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import lombok.experimental.UtilityClass;

/**
 * Constants for the physical algae intake subsystem using TalonFX motors. Contains PID constants
 * for different control modes and motor safety limits.
 */
@UtilityClass
public final class AlgaeIntakeTalonFXRealConstants {

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
      public static final double KG = 0.14;

      /** Static friction compensation */
      public static final double KS = 0.0;

      /** Velocity feedforward gain */
      public static final double KV = 0.11;
    }
  }

  /** Contains safety limit constants for the elevator motors. */
  public static final class MotorSafetyLimits {
    /** Maximum forward torque current limit in amperes */
    public static final Current TORQUE_FORWARD_AMP_LIMIT = Amps.of(35);

    /** Maximum reverse torque current limit in amperes */
    public static final Current TORQUE_REVERSE_AMP_LIMIT = Amps.of(-35);

    /** Maximum stator current limit in amperes */
    public static final Current STATOR_AMP_LIMIT = Amps.of(20);
  }

  /** Direction of the left motor rotation */
  public static final InvertedValue LEFT_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  /** Direction of the right motor rotation */
  public static final boolean RIGHT_MOTOR_INVERTED = true;

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final NeutralModeValue MECHANISM_NEUTRAL_MODE = NeutralModeValue.Brake;
}
