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

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import lombok.experimental.UtilityClass;

/**
 * Contains constants specific to the simulated implementation of the wrist. These values are used
 * to configure the physics simulation.
 */
@UtilityClass
public class WristTalonFXSimConstants {
  public static final class MotionMagicConstants {
    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(7);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(15);
    public static final double JERK = 0;
  }

  /** Moment of inertia of the wrist mechanism in kg⋅m² */
  public static final double INERTIA_KGM2 = 0.2809;

  /** Length of the wrist arm in meters */
  public static final double ARM_LENGTH = 0.3048;

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
  public static final double ENCODER_MAGNET_OFFSET = -0.225341796875;
}
