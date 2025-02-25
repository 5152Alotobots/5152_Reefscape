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

import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.measure.Current;
import lombok.experimental.UtilityClass;

/**
 * Constants for the physical algae intake subsystem using Neo 550 motors with SparkMax controllers.
 * Contains PID constants for different control modes and motor safety limits.
 */
@UtilityClass
public final class AlgaeIntakeSparkMaxRealConstants {

  /** Contains safety limit constants for the intake motors. */
  public static final class MotorSafetyLimits {
    /** Maximum stator current limit in amperes */
    public static final Current STATOR_AMP_LIMIT = Amps.of(25);
  }

  /** Direction of the left motor rotation (true = inverted) */
  public static final boolean LEFT_MOTOR_INVERTED = false;

  /** Direction of the right motor rotation (true = inverted) */
  public static final boolean RIGHT_MOTOR_INVERTED = true;

  /** Neutral mode (brake/coast) setting for the mechanism */
  public static final SparkBaseConfig.IdleMode MECHANISM_NEUTRAL_MODE =
      SparkBaseConfig.IdleMode.kBrake;

  /** Conversion factor from motor rotations to mechanism degrees */
  public static final double ROTATION_TO_DEGREES_CONVERSION = 360.0;
}
