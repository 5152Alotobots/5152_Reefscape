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
package frc.alotobots.reefscape.subsystems.wrist.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import lombok.experimental.UtilityClass;

/**
 * Contains TalonFX-specific constants for the real robot implementation of the wrist. Includes PID
 * values, motor configuration, and safety limits.
 */
@UtilityClass
public class WristTalonFXRealConstants {

  /** Contains PID and motion control constants for different control modes. */
  public static final class PIDConstants {
    /** TalonFX-specific PID and motion control constants for velocity mode. */
    public static final class VelocityPIDConstants {
      /** Position control proportional gain */
      public static final double KP = 0.1;

      /** Position control integral gain */
      public static final double KI = 0.0;

      /** Position control derivative gain */
      public static final double KD = 0.05;

      /** Acceleration feedforward gain */
      public static final double KA = 0.0;

      /** Gravity compensation gain */
      public static final double KG = 0.0;

      /** Static friction compensation */
      public static final double KS = 0.0;

      /** Velocity feedforward gain */
      public static final double KV = 32;
    }

    /** TalonFX-specific PID and motion control constants for Position mode. */
    public static final class PositionPIDConstants {
      /** Position control proportional gain */
      public static final double KP = 70;

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
  }

  public static final class MotionMagicConstants {
    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(3);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(3.5);
    public static final double JERK = 0;
  }

  /** Contains safety limit constants for the wrist motors. */
  public static final class MotorSafetyLimits {
    /** Maximum forward torque current limit in amperes */
    public static final Current TORQUE_FORWARD_AMP_LIMIT = Amps.of(35);

    /** Maximum reverse torque current limit in amperes */
    public static final Current TORQUE_REVERSE_AMP_LIMIT = Amps.of(-35);

    /** Maximum stator current limit in amperes */
    public static final Current STATOR_AMP_LIMIT = Amps.of(35);
  }

  /** Direction of the motor rotation */
  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;

  /** Direction of the encoder rotation */
  public static final SensorDirectionValue ENCODER_DIRECTION =
      SensorDirectionValue.CounterClockwise_Positive;

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final NeutralModeValue MECHANISM_NEUTRAL_MODE = NeutralModeValue.Brake;

  /** Gear ratio between motor rotation and mechanism rotation */
  public static final int ROTOR_TO_SENSOR_RATIO = 189;

  /** Magnet offset for the CANCoder absolute position */
  public static final double ENCODER_MAGNET_OFFSET = 0.0966796875;
}
