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

/**
 * Interface for logging state transitions in the localization system.
 *
 * <p>This interface provides a standardized way to record and monitor state machine transitions
 * within the localization system. It enables:
 *
 * <p>- Real-time monitoring of system behavior - Post-match analysis of state changes - Debugging
 * of unexpected transitions - Performance tracking and optimization - System health monitoring
 *
 * <p>Implementations should consider: - Logging to NetworkTables for real-time monitoring -
 * Persistent storage for post-match analysis - Timestamp recording for timing analysis - Context
 * preservation for debugging - Rate limiting for high-frequency transitions
 */
public interface StateTransitionLogger {

  /**
   * Records a state transition in the localization system.
   *
   * <p>This method is called whenever the system transitions between states. Implementations
   * should: - Record the timestamp of the transition - Log both the previous and new states -
   * Include relevant context if available - Handle rapid transitions appropriately - Ensure thread
   * safety
   *
   * <p>Common transitions to monitor: - UNINITIALIZED → RESETTING (Initial pose acquisition) -
   * RESETTING → QUEST_PRIMARY (Successful initialization) - QUEST_PRIMARY → TAG_BACKUP (Quest
   * failure or validation error) - TAG_BACKUP → QUEST_PRIMARY (Quest recovery) - Any → EMERGENCY
   * (System failure)
   *
   * @param from The state transitioning from
   * @param to The state transitioning to
   */
  void logTransition(LocalizationState.State from, LocalizationState.State to);
}
