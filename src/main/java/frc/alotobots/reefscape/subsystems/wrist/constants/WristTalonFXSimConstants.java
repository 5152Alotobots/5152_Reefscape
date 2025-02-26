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

import lombok.experimental.UtilityClass;

/**
 * Contains constants specific to the simulated implementation of the wrist. These values are used
 * to configure the physics simulation.
 */
@UtilityClass
public class WristTalonFXSimConstants {

  /** Gear ratio between motor rotation and mechanism rotation */
  public static final int ROTOR_TO_SENSOR_RATIO = 189;

  /** Moment of inertia of the wrist mechanism in kg⋅m² */
  public static final double INERTIA_KGM2 = 0.2809;

  /** Length of the wrist arm in meters */
  public static final double ARM_LENGTH = 0.3048;
}
