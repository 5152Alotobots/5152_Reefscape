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

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import lombok.experimental.UtilityClass;

/**
 * Contains all constants related to the algae intake subsystem. Organized into inner classes for
 * different categories of constants.
 */
@UtilityClass
public class AlgaeIntakeConstants {

  /** Contains threshold values for various algae intake operations. */
  public static final class Thresholds {
    /** Velocity threshold for considering the intake stopped (rad/s) */
    public static final double VELOCITY_STOPPED_THRESHOLD = 0.1;
  }

  /** Contains physical limits and safety thresholds for the algae intake. */
  public static final class Limits {

    /** Max velocity */
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(360);

    /** Maximum open loop percent output (global) */
    public static final double MAX_OPEN_LOOP_PERCENTAGE = 1.0;

    /** Maximum open loop intake percent output */
    public static final double MAX_OPEN_LOOP_INTAKE_PERCENTAGE = 0.7;

    /** Maximum open loop outtake percent output */
    public static final double MAX_OPEN_LOOP_OUTTAKE_PERCENTAGE = 1.0;
  }

  /** Setpoints for different algae intake states */
  public static final class Setpoints {
    public static final class OpenLoop {
      /** Default percent output for intake operation */
      public static final double INTAKE_PERCENTAGE = 0.7;

      /** Default percent output for eject operation */
      public static final double EJECT_PERCENTAGE = 1.0;
    }
  }
}
