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

import static edu.wpi.first.units.Units.Degrees;

import java.net.FileNameMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

public class WristConstants {
  public static final Angle MAX_ANGLE = Degrees.of(90);
  public static final Angle MIN_ANGLE = Degrees.of(0);

  public static final int ROTOR_TO_SENSOR_RATIO = 189;
  public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

  public static final double ENCODER_MAGNET_OFFSET = 0;
  public static final SensorDirectionValue ENCODER_DIRCTION_VALUE = SensorDirectionValue.Clockwise_Positive;
}
