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

import frc.alotobots.library.subsystems.bling.io.BlingIO.LoggedColor;
import org.littletonrobotics.junction.Logger;

/** Static class that manages the state of diagnostic LEDs across all subsystems. */
public class BlingDiagnosticManager {

  // LED Color definitions using LoggedColor
  public static final LoggedColor COLOR_OFF = new LoggedColor(0, 0, 0);
  public static final LoggedColor COLOR_GREEN = new LoggedColor(0, 255, 0);
  public static final LoggedColor COLOR_RED = new LoggedColor(255, 0, 0);
  public static final LoggedColor COLOR_YELLOW = new LoggedColor(255, 255, 0);
  public static final LoggedColor COLOR_BLUE = new LoggedColor(0, 0, 255);
  public static final LoggedColor COLOR_PURPLE = new LoggedColor(128, 0, 128);

  // Enums for state tracking
  public enum LocalizationState {
    QUEST_PRIMARY,
    TAG_BACKUP,
    RESETTING,
    EMERGENCY,
    UNINITIALIZED
  }

  public enum AutoStatus {
    SELECTED,
    NONE,
    INVALID
  }

  public enum ConfigStatus {
    COMPLETE,
    IN_PROGRESS,
    ERROR
  }

  // State variables
  private static LocalizationState localizationState = LocalizationState.UNINITIALIZED;
  private static boolean questConnected = false;
  private static boolean questInitialized = false;
  private static boolean tagsConnected = false;
  private static boolean tagsInitialized = false;
  private static AutoStatus autoStatus = AutoStatus.NONE;
  private static boolean driverStationConnected = false;
  private static boolean elevatorZeroed = false;
  private static boolean preloadDetected = false;
  private static ConfigStatus phoenixConfigStatus = ConfigStatus.IN_PROGRESS;

  // Track the current and previous colors for change detection
  private static LoggedColor[] currentColors = new LoggedColor[8];
  private static LoggedColor[] previousColors = new LoggedColor[8];

  static {
    // Initialize colors to OFF
    for (int i = 0; i < 8; i++) {
      currentColors[i] = COLOR_OFF;
      previousColors[i] = COLOR_OFF;
    }
  }

  // Setters for diagnostic values

  public static void setLocalizationState(LocalizationState state) {
    if (localizationState != state) {
      localizationState = state;
      logState();
    }
  }

  public static void setQuestStatus(boolean connected, boolean initialized) {
    if (questConnected != connected || questInitialized != initialized) {
      questConnected = connected;
      questInitialized = initialized;
      logState();
    }
  }

  public static void setAprilTagStatus(boolean connected, boolean initialized) {
    if (tagsConnected != connected || tagsInitialized != initialized) {
      tagsConnected = connected;
      tagsInitialized = initialized;
      logState();
    }
  }

  public static void setAutoStatus(AutoStatus status) {
    if (autoStatus != status) {
      autoStatus = status;
      logState();
    }
  }

  public static void setDriverStationConnected(boolean connected) {
    if (driverStationConnected != connected) {
      driverStationConnected = connected;
      logState();
    }
  }

  public static void setElevatorZeroed(boolean zeroed) {
    if (elevatorZeroed != zeroed) {
      elevatorZeroed = zeroed;
      logState();
    }
  }

  public static void setPreloadDetected(boolean detected) {
    if (preloadDetected != detected) {
      preloadDetected = detected;
      logState();
    }
  }

  public static void setPhoenixConfigStatus(ConfigStatus status) {
    if (phoenixConfigStatus != status) {
      phoenixConfigStatus = status;
      logState();
    }
  }

  // Color determination methods

  private static LoggedColor getLocalizationStateColor() {
    return switch (localizationState) {
      case QUEST_PRIMARY -> COLOR_GREEN;
      case TAG_BACKUP -> COLOR_BLUE;
      case RESETTING -> COLOR_YELLOW;
      default -> COLOR_RED;
    };
  }

  private static LoggedColor getQuestStatusColor() {
    if (questConnected && questInitialized) {
      return COLOR_GREEN;
    } else if (questConnected) {
      return COLOR_YELLOW;
    } else {
      return COLOR_RED;
    }
  }

  private static LoggedColor getAprilTagStatusColor() {
    if (tagsConnected && tagsInitialized) {
      return COLOR_GREEN;
    } else if (tagsConnected) {
      return COLOR_YELLOW;
    } else {
      return COLOR_RED;
    }
  }

  private static LoggedColor getAutoStatusColor() {
    return switch (autoStatus) {
      case SELECTED -> COLOR_GREEN;
      case NONE -> COLOR_YELLOW;
      case INVALID -> COLOR_RED;
    };
  }

  private static LoggedColor getDriverStationColor() {
    return driverStationConnected ? COLOR_GREEN : COLOR_RED;
  }

  private static LoggedColor getElevatorZeroedColor() {
    return elevatorZeroed ? COLOR_GREEN : COLOR_RED;
  }

  private static LoggedColor getPreloadDetectedColor() {
    return preloadDetected ? COLOR_GREEN : COLOR_RED;
  }

  private static LoggedColor getPhoenixConfigColor() {
    return switch (phoenixConfigStatus) {
      case COMPLETE -> COLOR_GREEN;
      case IN_PROGRESS -> COLOR_YELLOW;
      case ERROR -> COLOR_RED;
    };
  }

  // Public interface for getting LED colors

  /**
   * Get the current color for a specific LED index.
   *
   * @param ledIndex The LED index (0-7)
   * @return The LoggedColor for the LED
   */
  public static LoggedColor getLEDColor(int ledIndex) {
    if (ledIndex < 0 || ledIndex > 7) {
      return COLOR_OFF;
    }

    // Update the color based on the current state
    LoggedColor color =
        switch (ledIndex) {
          case 0 -> getLocalizationStateColor();
          case 1 -> getQuestStatusColor();
          case 2 -> getAprilTagStatusColor();
          case 3 -> getAutoStatusColor();
          case 4 -> getDriverStationColor();
          case 5 -> getElevatorZeroedColor();
          case 6 -> getPreloadDetectedColor();
          case 7 -> getPhoenixConfigColor();
          default -> COLOR_OFF;
        };

    currentColors[ledIndex] = color;
    return color;
  }

  /**
   * Check if the color for a specific LED has changed since the last check.
   *
   * @param ledIndex The LED index to check (0-7)
   * @return true if the color has changed
   */
  public static boolean hasColorChanged(int ledIndex) {
    if (ledIndex < 0 || ledIndex >= 8) {
      return false;
    }

    // Get the current color
    LoggedColor currentColor = getLEDColor(ledIndex);

    // Check if it's different from the previous color
    boolean changed = !currentColor.equals(previousColors[ledIndex]);

    // Update the previous color for next time
    previousColors[ledIndex] = currentColor;

    return changed;
  }

  /**
   * Get all LED colors at once.
   *
   * @return Array of LoggedColors for all LEDs
   */
  public static LoggedColor[] getAllLEDColors() {
    LoggedColor[] colors = new LoggedColor[8];
    for (int i = 0; i < 8; i++) {
      colors[i] = getLEDColor(i);
    }
    return colors;
  }

  /** Log the current diagnostic state. */
  private static void logState() {
    Logger.recordOutput("Diagnostics/LocalizationState", localizationState.toString());
    Logger.recordOutput("Diagnostics/QuestConnected", questConnected);
    Logger.recordOutput("Diagnostics/QuestInitialized", questInitialized);
    Logger.recordOutput("Diagnostics/TagsConnected", tagsConnected);
    Logger.recordOutput("Diagnostics/TagsInitialized", tagsInitialized);
    Logger.recordOutput("Diagnostics/AutoStatus", autoStatus.toString());
    Logger.recordOutput("Diagnostics/DriverStationConnected", driverStationConnected);
    Logger.recordOutput("Diagnostics/ElevatorZeroed", elevatorZeroed);
    Logger.recordOutput("Diagnostics/PreloadDetected", preloadDetected);
    Logger.recordOutput("Diagnostics/PhoenixConfigStatus", phoenixConfigStatus.toString());
  }
}
