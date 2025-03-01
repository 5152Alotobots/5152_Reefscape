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
package frc.alotobots.library.subsystems.bling.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.experimental.UtilityClass;

@UtilityClass
public class BlingUtil {
  /**
   * Schedules a command to run when a specific amount of time remains in the current match period.
   * Handles different timing behaviors based on FMS connection status. Only runs during teleop.
   *
   * @param command The command to schedule
   * @param timeRemaining Time remaining in the match when the command should trigger
   * @param tolerance Time tolerance to account for periodic execution
   * @param totalMatchTime Total duration of the match (typically 135 seconds for teleop)
   * @return The created Trigger object that will schedule the command
   */
  public static Trigger scheduleAtMatchTime(
      Command command, Time timeRemaining, Time tolerance, Time totalMatchTime) {
    // Timer to track teleop duration when not connected to FMS
    final Timer teleopTimer = new Timer();
    final boolean[] teleopStarted = {false};

    return new Trigger(
            () -> {
              // Only trigger during teleop
              boolean isTeleop = DriverStation.isTeleopEnabled();

              if (!isTeleop) {
                // Reset teleop tracking when not in teleop
                teleopStarted[0] = false;
                return false;
              } else if (!teleopStarted[0]) {
                // Start tracking teleop time when we first enter teleop
                teleopTimer.reset();
                teleopTimer.start();
                teleopStarted[0] = true;
              }

              // Get current match time based on FMS connection
              Time currentMatchTime;

              // If connected to FMS, use the countdown timing from DriverStation
              if (DriverStation.isFMSAttached()) {
                double matchTimeSeconds = DriverStation.getMatchTime();
                currentMatchTime = Units.Seconds.of(matchTimeSeconds);
              } else {
                // Not connected to FMS - calculate remaining time based on elapsed teleop time
                double elapsedTime = teleopTimer.get();
                double remainingTime = totalMatchTime.in(Units.Seconds) - elapsedTime;
                currentMatchTime = Units.Seconds.of(remainingTime);
              }

              // Return true when we're within tolerance of the target time
              return currentMatchTime.gte(timeRemaining.minus(tolerance))
                  && currentMatchTime.lte(timeRemaining.plus(tolerance));
            })
        .onTrue(command);
  }

  /**
   * Schedules a command to run when a specific amount of time remains in the current match period.
   * Uses default tolerance of 0.1 seconds and default teleop time of 135 seconds.
   *
   * @param command The command to schedule
   * @param timeRemaining Time remaining in the match when the command should trigger
   * @return The created Trigger object that will schedule the command
   */
  public static Trigger scheduleAtMatchTime(Command command, Time timeRemaining) {
    return scheduleAtMatchTime(
        command,
        timeRemaining,
        Units.Seconds.of(0.1),
        Units.Seconds.of(135) // Standard teleop duration
        );
  }
}
