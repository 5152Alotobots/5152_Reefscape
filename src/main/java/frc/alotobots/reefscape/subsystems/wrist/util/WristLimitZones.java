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
package frc.alotobots.reefscape.subsystems.wrist.util;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import lombok.experimental.UtilityClass;

/**
 * Manages wrist limits based on elevator height zones. Contains both the zone definition and zone
 * management functionality.
 */
@UtilityClass
public class WristLimitZones {

  /** Defines a zone of elevator heights with corresponding wrist rotation limits. */
  public static class WristLimitZone {
    /** Minimum elevator height for this zone */
    private final Distance minElevatorHeight;

    /** Maximum elevator height for this zone */
    private final Distance maxElevatorHeight;

    /**
     * Minimum allowed wrist angle when in this zone -- GETTER -- Gets the minimum allowed wrist
     * angle for this zone.
     *
     * @return The minimum wrist angle
     */
    @Getter private final Angle minWristAngle;

    /**
     * Maximum allowed wrist angle when in this zone -- GETTER -- Gets the maximum allowed wrist
     * angle for this zone.
     *
     * @return The maximum wrist angle
     */
    @Getter private final Angle maxWristAngle;

    /**
     * Creates a new WristLimitZone.
     *
     * @param minElevatorHeight The minimum elevator height this zone applies to
     * @param maxElevatorHeight The maximum elevator height this zone applies to
     * @param minWristAngle The minimum allowed wrist angle in this zone
     * @param maxWristAngle The maximum allowed wrist angle in this zone
     */
    public WristLimitZone(
        Distance minElevatorHeight,
        Distance maxElevatorHeight,
        Angle minWristAngle,
        Angle maxWristAngle) {
      this.minElevatorHeight = minElevatorHeight;
      this.maxElevatorHeight = maxElevatorHeight;
      this.minWristAngle = minWristAngle;
      this.maxWristAngle = maxWristAngle;
    }

    /**
     * Checks if the provided elevator height falls within this zone.
     *
     * @param elevatorHeight The current elevator height
     * @return true if the height is within this zone's range (inclusive of min/max)
     */
    public boolean isInZone(Distance elevatorHeight) {
      return elevatorHeight.gte(minElevatorHeight) && elevatorHeight.lte(maxElevatorHeight);
    }
  }

  /** Default zone used when no other zone matches */
  public static final WristLimitZone DEFAULT_ZONE =
      new WristLimitZone(
          MIN_HEIGHT, // Min elevator height
          MAX_HEIGHT, // Max elevator height
          MIN_ANGLE, // Min wrist angle
          MAX_ANGLE // Max wrist angle
          );

  /** List of defined limit zones */
  private static final List<WristLimitZone> zones = new ArrayList<>();

  static {
    // Initialize zones - these should cover the entire range of elevator heights

    // Zone 1: Bottom zone
    zones.add(
        new WristLimitZone(
            MIN_HEIGHT, // Min elevator height
            Meters.of(.35), // Max elevator height
            Degrees.of(3), // Min wrist angle
            Degrees.of(95) // Max wrist angle
            ));

    zones.add(
        new WristLimitZone(
            Meters.of(.35), // Min elevator height
            Meters.of(0.433), // Max elevator height
            Degrees.of(-13), // Min wrist angle
            Degrees.of(110) // Max wrist angle
            ));

    zones.add(
        new WristLimitZone(
            Meters.of(.433), // Min elevator height
            Meters.of(0.55), // Max elevator height
            MIN_ANGLE, // Min wrist angle
            Degrees.of(110) // Max wrist angle
            ));

    // Zone 2: Middle zone
    zones.add(
        new WristLimitZone(
            Meters.of(0.55), // Min elevator height
            Meters.of(1.4), // Max elevator height
            MIN_ANGLE, // Min wrist angle (full range)
            Degrees.of(105) // Max wrist angle
            ));

    // Zone 3: Mid-Top zone
    zones.add(
        new WristLimitZone(
            Meters.of(1.1), // Min elevator height
            Meters.of(1.6), // Max elevator height
            MIN_ANGLE, // Min wrist angle (full range)
            Degrees.of(110) // Max wrist angle
            ));

    // Zone 4: Mid-Top zone
    zones.add(
        new WristLimitZone(
            Meters.of(1.6), // Min elevator height
            MAX_HEIGHT, // Max elevator height
            MIN_ANGLE, // Min wrist angle (full range)
            MAX_ANGLE // Max wrist angle
            ));
  }

  /**
   * Finds the applicable wrist limit zone for the given elevator height.
   *
   * @param elevatorHeight The current elevator height
   * @return The matching WristLimitZone, or the DEFAULT_ZONE if no zone matches
   */
  public static WristLimitZone findZone(Distance elevatorHeight) {
    for (WristLimitZone zone : zones) {
      if (zone.isInZone(elevatorHeight)) {
        return zone;
      }
    }
    return DEFAULT_ZONE;
  }

  /**
   * Gets the minimum wrist angle limit for the given elevator height.
   *
   * @param elevatorHeight The current elevator height
   * @return The minimum allowed wrist angle
   */
  public static Angle getMinAngle(Distance elevatorHeight) {
    return findZone(elevatorHeight).getMinWristAngle();
  }

  /**
   * Gets the maximum wrist angle limit for the given elevator height.
   *
   * @param elevatorHeight The current elevator height
   * @return The maximum allowed wrist angle
   */
  public static Angle getMaxAngle(Distance elevatorHeight) {
    return findZone(elevatorHeight).getMaxWristAngle();
  }

  /**
   * Adds a new zone to the collection.
   *
   * @param zone The zone to add
   */
  public static void addZone(WristLimitZone zone) {
    zones.add(zone);
  }

  /**
   * Determines if forward (positive) limit should be active based on current wrist angle and
   * target. Forward limit should be active when approaching the maximum allowed angle.
   *
   * @param currentAngle The current angle of the wrist
   * @param maxAngle The maximum allowed angle based on elevator height
   * @return true if forward limit should be active
   */
  public static boolean shouldLimitForwardMotion(Angle currentAngle, Angle maxAngle) {
    // If current angle is within threshold of max angle, activate limit
    return ElevatorConstants.Limits.LIMITS_ENABLED && currentAngle.gte(maxAngle);
  }

  /**
   * Determines if reverse (negative) limit should be active based on current wrist angle and
   * target. Reverse limit should be active when approaching the minimum allowed angle.
   *
   * @param currentAngle The current angle of the wrist
   * @param minAngle The minimum allowed angle based on elevator height
   * @return true if reverse limit should be active
   */
  public static boolean shouldLimitReverseMotion(Angle currentAngle, Angle minAngle) {
    // If current angle is within threshold of min angle, activate limit
    return ElevatorConstants.Limits.LIMITS_ENABLED && currentAngle.lte(minAngle);
  }
}
