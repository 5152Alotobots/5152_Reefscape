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
package frc.alotobots.reefscape.subsystems.autocycle.constants;

import edu.wpi.first.math.util.Units;

/**
 * Constants used for the AutoCycle subsystem's path following and alignment functionality.
 * These values determine when the robot is considered "close enough" to its target position.
 */
public class AutoCycleConstants {
  /**
   * Translation tolerance in meters. Specifies how far from the end of the path translation
   * is considered "close enough" for alignment purposes.
   */
  public static final double ALIGNMENT_TRANSLATION_TOLERANCE = 0.2;

  /**
   * Rotation tolerance in radians. Specifies how far from the end of the path rotation
   * is considered "close enough" for alignment purposes.
   */
  public static final double ALIGNMENT_ROTATION_TOLERANCE = Units.degreesToRadians(15);
}