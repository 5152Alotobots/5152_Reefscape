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

import org.littletonrobotics.junction.Logger;

/**
 * Static class that manages the state of diagnostic LEDs across all subsystems.
 * Subsystems update their status through this class, and the BlingSubsystem reads
 * from it to control LED colors.
 */
public class BlingDiagnosticManager {

  // LED Status Enums
  public enum LEDStatus {
    OFF,
    GREEN,
    YELLOW,
    RED,
    BLUE,
    PURPLE
  }

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

  // Diagnostic State Variables
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

  // Internal flags for change detection
  private static boolean[] changedFlags = new boolean[8];
  private static LEDStatus[] currentLEDStatus = new LEDStatus[8];

  static {
    // Initialize all LEDs as OFF
    for (int i = 0; i < 8; i++) {
      currentLEDStatus[i] = LEDStatus.OFF;
      changedFlags[i] = true; // Mark all as changed initially
    }
  }

  // Setters for each diagnostic value - to be called by subsystems

  public static void setLocalizationState(LocalizationState state) {
    if (localizationState != state) {
      localizationState = state;
      changedFlags[0] = true;
      logState();
    }
  }

  public static void setQuestStatus(boolean connected, boolean initialized) {
    if (questConnected != connected || questInitialized != initialized) {
      questConnected = connected;
      questInitialized = initialized;
      changedFlags[1] = true;
      logState();
    }
  }

  public static void setAprilTagStatus(boolean connected, boolean initialized) {
    if (tagsConnected != connected || tagsInitialized != initialized) {
      tagsConnected = connected;
      tagsInitialized = initialized;
      changedFlags[2] = true;
      logState();
    }
  }

  public static void setAutoStatus(AutoStatus status) {
    if (autoStatus != status) {
      autoStatus = status;
      changedFlags[3] = true;
      logState();
    }
  }

  public static void setDriverStationConnected(boolean connected) {
    if (driverStationConnected != connected) {
      driverStationConnected = connected;
      changedFlags[4] = true;
      logState();
    }
  }

  public static void setElevatorZeroed(boolean zeroed) {
    if (elevatorZeroed != zeroed) {
      elevatorZeroed = zeroed;
      changedFlags[5] = true;
      logState();
    }
  }

  public static void setPreloadDetected(boolean detected) {
    if (preloadDetected != detected) {
      preloadDetected = detected;
      changedFlags[6] = true;
      logState();
    }
  }

  public static void setPhoenixConfigStatus(ConfigStatus status) {
    if (phoenixConfigStatus != status) {
      phoenixConfigStatus = status;
      changedFlags[7] = true;
      logState();
    }
  }

  // Getters used by BlingSubsystem

  public static LEDStatus getLED0Status() {
    LEDStatus status;
    switch (localizationState) {
      case QUEST_PRIMARY:
        status = LEDStatus.GREEN;
        break;
      case TAG_BACKUP:
        status = LEDStatus.BLUE;
        break;
      case RESETTING:
        status = LEDStatus.YELLOW;
        break;
      case EMERGENCY:
      case UNINITIALIZED:
      default:
        status = LEDStatus.RED;
        break;
    }
    currentLEDStatus[0] = status;
    return status;
  }

  public static LEDStatus getLED1Status() {
    LEDStatus status;
    if (questConnected && questInitialized) {
      status = LEDStatus.GREEN;
    } else if (questConnected) {
      status = LEDStatus.YELLOW;
    } else {
      status = LEDStatus.RED;
    }
    currentLEDStatus[1] = status;
    return status;
  }

  public static LEDStatus getLED2Status() {
    LEDStatus status;
    if (tagsConnected && tagsInitialized) {
      status = LEDStatus.GREEN;
    } else if (tagsConnected) {
      status = LEDStatus.YELLOW;
    } else {
      status = LEDStatus.RED;
    }
    currentLEDStatus[2] = status;
    return status;
  }

  public static LEDStatus getLED3Status() {
    LEDStatus status;
    switch (autoStatus) {
      case SELECTED:
        status = LEDStatus.GREEN;
        break;
      case NONE:
        status = LEDStatus.YELLOW;
        break;
      case INVALID:
        status = LEDStatus.RED;
        break;
      default:
        status = LEDStatus.OFF;
    }
    currentLEDStatus[3] = status;
    return status;
  }

  public static LEDStatus getLED4Status() {
    LEDStatus status = driverStationConnected ? LEDStatus.GREEN : LEDStatus.RED;
    currentLEDStatus[4] = status;
    return status;
  }

  public static LEDStatus getLED5Status() {
    LEDStatus status = elevatorZeroed ? LEDStatus.GREEN : LEDStatus.RED;
    currentLEDStatus[5] = status;
    return status;
  }

  public static LEDStatus getLED6Status() {
    LEDStatus status = preloadDetected ? LEDStatus.GREEN : LEDStatus.RED;
    currentLEDStatus[6] = status;
    return status;
  }

  public static LEDStatus getLED7Status() {
    LEDStatus status;
    switch (phoenixConfigStatus) {
      case COMPLETE:
        status = LEDStatus.GREEN;
        break;
      case IN_PROGRESS:
        status = LEDStatus.YELLOW;
        break;
      case ERROR:
        status = LEDStatus.RED;
        break;
      default:
        status = LEDStatus.OFF;
    }
    currentLEDStatus[7] = status;
    return status;
  }

  // Utility methods

  /**
   * Check if the status for a specific LED has changed
   * 
   * @param ledIndex The LED index to check (0-7)
   * @return true if status has changed since last check
   */
  public static boolean hasStatusChanged(int ledIndex) {
    if (ledIndex < 0 || ledIndex >= 8) return false;
    boolean changed = changedFlags[ledIndex];
    changedFlags[ledIndex] = false; // Reset the flag once checked
    return changed;
  }

  /**
   * Get the current status for any LED by index
   * 
   * @param ledIndex The LED index (0-7)
   * @return The current status for that LED
   */
  public static LEDStatus getLEDStatus(int ledIndex) {
    switch (ledIndex) {
      case 0: return getLED0Status();
      case 1: return getLED1Status();
      case 2: return getLED2Status();
      case 3: return getLED3Status();
      case 4: return getLED4Status();
      case 5: return getLED5Status();
      case 6: return getLED6Status();
      case 7: return getLED7Status();
      default: return LEDStatus.OFF;
    }
  }

  /**
   * Gets all LED statuses at once
   * 
   * @return Array of current LED statuses
   */
  public static LEDStatus[] getAllLEDStatus() {
    LEDStatus[] statuses = new LEDStatus[8];
    for (int i = 0; i < 8; i++) {
      statuses[i] = getLEDStatus(i);
    }
    return statuses;
  }

  /**
   * Log the current diagnostic state via AdvantageKit
   */
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
    
    // Log the LED statuses too
    for (int i = 0; i < 8; i++) {
      Logger.recordOutput("Diagnostics/LED" + i, getLEDStatus(i).toString());
    }
  }
}