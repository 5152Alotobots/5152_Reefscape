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
package frc.alotobots.reefscape.subsystems.coralIntake.constants;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.measure.Current;
import lombok.experimental.UtilityClass;

/**
 * Constants for the physical intake subsystem using Vortex motors. Contains PID constants for
 * different control modes and motor safety limits.
 */
@UtilityClass
public final class CoralIntakeVortexRealConstants {

  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {
    /** Vortex-specific PID and motion control constants for velocity (Velocity mode). */
    public static final class VelocityPIDConstants {
      /** Velocity control proportional gain */
      public static final double KP = 0.1;

      /** Velocity control integral gain */
      public static final double KI = 0.0;

      /** Velocity control derivative gain */
      public static final double KD = 0.0;

      /** Velocity control feedforward gain (Important for REV's half-baked velocity control) */
      public static final double KF = 0.0;
    }
  }

  /** Contains safety limit constants for the intake motor. */
  public static final class MotorSafetyLimits {
    /** Maximum forward torque current limit in amperes */
    public static final Current TORQUE_FORWARD_AMP_LIMIT = Amps.of(35);

    /** Maximum reverse torque current limit in amperes */
    public static final Current TORQUE_REVERSE_AMP_LIMIT = Amps.of(-35);

    /** Maximum stator current limit in amperes */
    public static final Current STATOR_AMP_LIMIT = Amps.of(35);
  }

  /** Direction of the left motor rotation */
  public static final boolean MOTOR_DIRECTION = false; // CCW+

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final SparkBaseConfig.IdleMode MECHANISM_NEUTRAL_MODE =
      SparkBaseConfig.IdleMode.kBrake;
}
