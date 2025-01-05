/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.localizationfusion;

import lombok.experimental.UtilityClass;

/**
 * Constants used by the LocalizationFusion subsystem for robot pose estimation and tracking. Groups
 * related constants into inner classes for better organization.
 */
@UtilityClass
public class LocalizationFusionConstants {

  /**
   * Constants related to pose validation and thresholds. Defines distance and rotation thresholds
   * for validating poses from different sources.
   */
  @UtilityClass
  public static class ValidationThresholds {
    /** Maximum acceptable difference between AprilTag and Quest poses for validation (meters). */
    public static final double APRILTAG_VALIDATION_THRESHOLD = 1.0;

    /** Stricter threshold used during initialization phase for validating poses (meters). */
    public static final double INIT_VALIDATION_THRESHOLD = 0.3;

    /** Maximum allowed pose change during disabled state to trigger recalibration (meters). */
    public static final double DISABLED_RECALIBRATION_THRESHOLD = 0.05;

    /** Maximum allowed rotation change between poses (degrees). */
    public static final double MAX_ROTATION_CHANGE_DEGREES = 35.0;
  }

  /**
   * Constants related to the auto-realignment feature for correcting Quest drift using AprilTags.
   * This system monitors robot stability and pose error to trigger automatic realignments when the
   * robot is stationary and significant drift is detected.
   */
  @UtilityClass
  public class AutoRealignConstants {
    /** Whether auto-realignment should be enabled. */
    public static final boolean ENABLED = true;

    /** Threshold for auto-realignment when pose error exceeds this value (meters). */
    public static final double THRESHOLD = 0.1;

    /** Maximum pose change over stability period to be considered stable (meters). */
    public static final double MAX_MOVEMENT = 0.05;

    /** Time robot must be stable before realigning (seconds). */
    public static final double STABILITY_TIME = 1.0;

    /** Minimum time between auto-realignments (seconds). */
    public static final double COOLDOWN = 10.0;
  }

  /**
   * Constants related to timing and update intervals. Defines various timeouts, intervals, and
   * timing windows used in the localization system.
   */
  @UtilityClass
  public static class Timing {
    /** Update interval matching Quest's native 120Hz update rate (seconds). */
    public static final double POSE_UPDATE_INTERVAL = 1.0 / 120.0;

    /** Time required for Quest initialization to complete (seconds). */
    public static final double QUEST_INIT_TIMEOUT = 2.0;

    /** Time required for AprilTag initialization to complete (seconds). */
    public static final double TAG_INIT_TIMEOUT = 1.0;

    /** Minimum time required to validate initial pose stability (seconds). */
    public static final double INITIAL_POSE_STABILITY_TIME = 15.0;

    /** Maximum time to wait for reset sequence to complete (seconds). */
    public static final double RESET_TIMEOUT = 5.0;

    /** Time after match start to consider it in mid-match state (seconds). */
    public static final double MATCH_STARTUP_PERIOD_SECONDS = 5.0;
  }

  /**
   * Constants related to initialization requirements and validation counts. Defines minimum update
   * counts and multipliers for system initialization.
   */
  @UtilityClass
  public static class InitializationRequirements {
    /** Minimum number of valid Quest updates required for successful initialization. */
    public static final int MIN_QUEST_VALID_UPDATES = 10;

    /** Minimum number of valid AprilTag updates required for successful initialization. */
    public static final int MIN_TAG_VALID_UPDATES = 3;

    /** Multiplier for extended Quest initialization grace period. */
    public static final double QUEST_INIT_GRACE_MULTIPLIER = 3.0;
  }
}
