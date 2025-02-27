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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import lombok.experimental.UtilityClass;

/**
 * Contains all constants related to the wrist subsystem. Organized into inner classes for different
 * categories of constants.
 */
@UtilityClass
public class WristConstants {

  /** Contains threshold values for various wrist operations. */
  public static final class Thresholds {
    /** Acceptable PID error that will classify as "at position" */
    public static final Angle AT_TARGET_ANGLE_POSITION_THRESHOLD = Degrees.of(3);

    /** How long the wrist must be "at position" to classify as "at position" */
    public static final Time AT_TARGET_ANGLE_TIME_THRESHOLD = Seconds.of(.2);
  }

  /** Contains physical limits and safety thresholds for the wrist. */
  public static final class Limits {
    /** Maximum allowed angle */
    public static final Angle MAX_ANGLE = Degrees.of(125);

    /** Minimum allowed angle */
    public static final Angle MIN_ANGLE = Degrees.of(-10);

    /** Maximum open loop percent output */
    public static final double MAX_OPEN_LOOP_PERCENTAGE = 0.5;

    /** Max speed (magnitude) */
    public static final AngularVelocity MAX_SPEED = DegreesPerSecond.of(90);

    /** Enable Limits */
    public static final boolean LIMITS_ENABLED = true;
  }

  /** Contains position setpoints for different wrist states. */
  public static final class Setpoints {
    /** Angle when wrist is fully retracted/stowed */
    public static final Angle STOWED = Degrees.of(115);

    /** Angle when wrist is moving to a position. Safe position */
    public static final Angle CRUISE = Degrees.of(90);

    /** Angle for picking up from coral station */
    public static final Angle CORAL_STATION = Degrees.of(33);

    /** Angle for L4 placement */
    public static final Angle L4_PLACE = Degrees.of(110);

    /** Angle for L3 placement (Equal to the angle for L2 placement) */
    public static final Angle L3_PLACE = Degrees.of(120);

    /** Angle for L2 placement (Equal to the angle for L3 placement) */
    public static final Angle L2_PLACE = Degrees.of(115);

    /** Angle for L1 placement */
    public static final Angle L1_PLACE = Degrees.of(105);

    /** Angle for ground intake */
    public static final Angle GROUND_INTAKE = Degrees.of(-7);
  }
}
