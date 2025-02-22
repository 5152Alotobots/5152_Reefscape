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
package frc.alotobots.reefscape.subsystems.elevator.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import lombok.experimental.UtilityClass;

/**
 * Constants for the Elevator subsystem. Contains all configuration values, conversion factors, and
 * physical limits. All values use WPILib's units library for type safety.
 */
@UtilityClass
public final class ElevatorConstants {

  public static final class Thresholds {
    /** Acceptable PID error that will classify as "at position" */
    public static final Distance AT_TARGET_HEIGHT_POSITION_THRESHOLD = Meters.of(.02);

    /** How long the elevator must be "at position" to classify as "at position" */
    public static final Time AT_TARGET_HEIGHT_TIME_THRESHOLD = Seconds.of(.2);
  }

  /** Physical limits and safety thresholds */
  public static final class Limits {
    /** Maximum allowed height */
    public static final Distance MAX_HEIGHT = Meters.of(1.80);

    /** Minimum allowed height */
    public static final Distance MIN_HEIGHT = Meters.of(0.253311);

    /** Maximum open loop percent output */
    public static final double MAX_OPEN_LOOP_PERCENTAGE = 0.5;

    /** Max speed (magnitude) */
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(0.5);

    /** Enable Limits */
    public static final boolean LIMITS_ENABLED = true;
  }

  /** Position setpoints for different elevator states */
  public static final class Setpoints {
    /** Height when elevator is fully retracted/stowed */
    public static final Distance STOWED = Meters.of(0.3);

    /** Height for picking up from coral station */
    public static final Distance CORAL_STATION = Meters.of(0.80);

    /** Height for L4 placement */
    public static final Distance L4_PLACE = Meters.of(1.78);

    /** Height for L3 placement */
    public static final Distance L3_PLACE = Meters.of(1.2);

    /** Height for L2 placement */
    public static final Distance L2_PLACE = Meters.of(0.81);

    /** Height for L1 placement */
    public static final Distance L1_PLACE = Meters.of(0.457);
  }
}
