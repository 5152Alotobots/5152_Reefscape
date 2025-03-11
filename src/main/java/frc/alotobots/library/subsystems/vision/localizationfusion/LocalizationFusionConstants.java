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
package frc.alotobots.library.subsystems.vision.localizationfusion;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.List;
import java.util.Map;

import edu.wpi.first.units.measure.Time;
import lombok.experimental.UtilityClass;

import static edu.wpi.first.units.Units.Seconds;

@UtilityClass
public class LocalizationFusionConstants {
  public static final Map<LocalizationType, Integer> SOURCE_TYPE_PRIORITIES =
      Map.of(
          LocalizationType.QUEST, 0,
          LocalizationType.MULTI_TAG, 1,
          LocalizationType.SINGLE_TAG, 2,
          LocalizationType.ODOMETRY, 3,
          LocalizationType.NONE, 4);

  /** If the source matches any of these states, automatically fall back to a lower
   * state if available
   * */
  public static final List<LocalizationSourceState> FALLBACK_SOURCE_STATES =
          List.of(
                  LocalizationSourceState.FAULT,
                  LocalizationSourceState.DISABLED
          );

  public Time ELASTIC_NOTIFICATION_DURATION = Seconds.of(0.5);

  class Logging {
    public static final String AKIT_BASE_PATH = "LocalizationFusion";
    public static final String AKIT_SOURCE_CHANGED_PATH = AKIT_BASE_PATH + "/SourceChanged";
    public static final String AKIT_SOURCE_TYPE_PATH = AKIT_BASE_PATH + "/SourceType";
    public static final String AKIT_SOURCE_STATE_PATH = AKIT_BASE_PATH + "/SourceState";
    public static final String AKIT_SOURCE_POSE_PATH = AKIT_BASE_PATH + "/SourcePose";
    public static final String AKIT_SOURCE_STD_DEVS_PATH = AKIT_BASE_PATH + "/SourceStdDevs";
    public static final String AKIT_SOURCE_TIMESTAMP_PATH = AKIT_BASE_PATH + "/SourceTimestamp";

  }
}
