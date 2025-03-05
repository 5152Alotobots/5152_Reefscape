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
package frc.alotobots.library.subsystems.vision.oculus.util;

/**
 * Constants for Quest communication status codes.
 *
 * <p>This class defines the status codes used in communication between the robot and Quest headset.
 */
public class OculusStatus {
  /** Status code indicating Quest is ready for commands */
  public static final int STATUS_READY = 0;

  /** Status code indicating error state */
  public static final int STATUS_ERROR = -1;

  /** Status code indicating Quest is disconnecting */
  public static final int STATUS_DISCONNECT = -2;

  /** Status code indicating successful pose reset */
  public static final int STATUS_POSE_RESET_COMPLETE = 98;

  /** Status code indicating successful heading reset */
  public static final int STATUS_HEADING_RESET_COMPLETE = 99;

  /** Status code indicating ping response */
  public static final int STATUS_PING_RESPONSE = 97;

  /** Status code indicating successful transform update */
  public static final int STATUS_TRANSFORM_SUCCESS = 96;
}
