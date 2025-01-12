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

import lombok.experimental.UtilityClass;

@UtilityClass
public final class OculusStatus {
  /** Status indicating system is ready for commands */
  public static final int STATUS_READY = 0;

  /** Status indicating heading reset completion */
  public static final int STATUS_HEADING_RESET_COMPLETE = 99;

  /** Status indicating pose reset completion */
  public static final int STATUS_POSE_RESET_COMPLETE = 98;

  /** Status indicating ping response receipt */
  public static final int STATUS_PING_RESPONSE = 97;
}
