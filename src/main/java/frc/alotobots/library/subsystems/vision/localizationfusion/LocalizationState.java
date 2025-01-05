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

import lombok.Getter;

/**
 * Manages the state machine for the robot localization system.
 *
 * <p>This class implements a hierarchical state machine that controls transitions between different
 * pose estimation sources based on their availability and reliability. The system follows this
 * hierarchy:
 *
 * <p>1. Quest SLAM as primary source (120Hz updates) 2. AprilTags as backup source 3. Odometry as
 * emergency fallback
 *
 * <p>State transitions are logged for debugging and monitoring purposes.
 */
public class LocalizationState {

  /**
   * Represents the possible states of the localization system. States are ordered by initialization
   * sequence and fallback hierarchy.
   */
  public enum State {
    /**
     * System is waiting for initial pose acquisition. Valid transitions: - To RESETTING when
     * initial AprilTag pose is acquired
     */
    UNINITIALIZED("Waiting for initial pose"),

    /**
     * System is resetting to a reference pose. Valid transitions: - To QUEST_PRIMARY when reset
     * completes successfully - To TAG_BACKUP if Quest becomes unavailable during reset
     */
    RESETTING("Resetting to reference pose"),

    /**
     * System is using Quest SLAM as primary pose source. Valid transitions: - To TAG_BACKUP if
     * Quest connection is lost or validation fails - To EMERGENCY if both Quest and AprilTags are
     * unavailable
     */
    QUEST_PRIMARY("Using Quest as primary source"),

    /**
     * System is using AprilTags as backup pose source. Valid transitions: - To QUEST_PRIMARY when
     * Quest connection is restored - To EMERGENCY if AprilTags become unavailable
     */
    TAG_BACKUP("Using AprilTags as backup"),

    /**
     * System is using only wheel odometry for pose estimation. Valid transitions: - To TAG_BACKUP
     * when AprilTags become available - To QUEST_PRIMARY when Quest connection is restored
     */
    EMERGENCY("Using only odometry");

    @Getter private final String description;

    /**
     * Creates a new State with the specified description.
     *
     * @param description Human-readable description of the state
     */
    State(String description) {
      this.description = description;
    }
  }

  /** Current state of the localization system */
  @Getter private State currentState = State.UNINITIALIZED;

  /** Logger for state transitions */
  @Getter private final StateTransitionLogger transitionLogger;

  /**
   * Creates a new LocalizationState instance.
   *
   * @param logger Logger to record state transitions
   */
  public LocalizationState(StateTransitionLogger logger) {
    this.transitionLogger = logger;
  }

  /**
   * Attempts to transition to a new state. The transition will be logged through the
   * StateTransitionLogger. Invalid transitions will be logged but not executed.
   *
   * @param newState The target state to transition to
   */
  public void transitionTo(State newState) {
    if (newState != currentState) {
      // Log state transition
      transitionLogger.logTransition(currentState, newState);

      // Execute transition
      currentState = newState;
    }
  }

  /**
   * Checks if the system is currently in any of the specified states. This is useful for checking
   * multiple valid states at once.
   *
   * @param states Variable number of states to check against
   * @return true if current state matches any of the specified states
   */
  public boolean isInState(State... states) {
    for (State state : states) {
      if (currentState == state) return true;
    }
    return false;
  }
}
