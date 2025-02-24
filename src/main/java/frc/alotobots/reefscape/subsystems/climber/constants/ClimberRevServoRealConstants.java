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
package frc.alotobots.reefscape.subsystems.climber.constants;

import com.revrobotics.servohub.ServoChannel.ChannelId;

public final class ClimberRevServoRealConstants {
  public static final ChannelId PLUNGER_SERVO_ID = ChannelId.kChannelId2;
  public static final ChannelId LOCKING_SERVO_ID = ChannelId.kChannelId0;

  public static final int CAGE_SWITCH_1_ID = 0;
  public static final int CAGE_SWITCH_2_ID = 1;

  public static final int PLUNGER_SERVO_0_PW = 650;
  public static final int PLUNGER_SERVO_180_PW = 2500;
  public static final int LOCKING_SERVO_OPEN_PW = 1773;
  public static final int LOCKING_SERVO_CLOSED_PW = 1446;
}
