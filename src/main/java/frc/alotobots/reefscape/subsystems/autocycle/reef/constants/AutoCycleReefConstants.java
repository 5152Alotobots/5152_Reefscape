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
package frc.alotobots.reefscape.subsystems.autocycle.reef.constants;

import edu.wpi.first.math.util.Units;

public class AutoCycleReefConstants {
  /** How far from the end of the path translation is considered "close" */
  public static final double ALIGNMENT_TRANSLATION_TOLERANCE = 0.2;

  /** How far from the end of the path rotation is considered "close" */
  public static final double ALIGNMENT_ROTATION_TOLERANCE = Units.degreesToRadians(15);
}
