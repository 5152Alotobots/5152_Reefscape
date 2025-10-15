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
package frc.alotobots.library.subsystems.vision.questnav.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lombok.experimental.UtilityClass;

/**
 * Constants used by the QuestNav Quest navigation subsystem. Contains configuration values for
 * physical setup and operation parameters.
 */
@UtilityClass
public class QuestNavConstants {
  public static final double BATTERY_LOW_PERCENT = 20;

  public static final double BATTERY_CRITICAL_PERCENT = 10;

  /**
   * Transform from the robot center to the headset. Coordinate system: - X: Positive is forwards -
   * Y: Positive is left - Rotation: Positive is counter-clockwise
   */
  public static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(0.153, -0.26, 0.0, new Rotation3d(Rotation2d.fromDegrees(-90)));

  /**
   * Standard deviations representing how much we "trust" the position from the QuestNav. By
   * default, the Quest 3 provides sub-centimeter accuracy. Values represent: [0]: X position trust
   * (50mm) [1]: Y position trust (50mm) [2]: Rotation trust (~2.87 degrees)
   */
  public static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
          0.02, // Trust down to 50mm
          0.02, // Trust down to 50mm
          0.0872665 // 5deg
          );

  // Strictly used for field dimensions
  public static AprilTagFieldLayout QUESTNAV_FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
