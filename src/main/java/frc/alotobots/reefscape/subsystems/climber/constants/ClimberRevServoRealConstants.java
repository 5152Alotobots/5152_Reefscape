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

import com.revrobotics.servohub.ServoChannel;

/**
 * Constants for the real servo configuration in the climber subsystem. This class contains the IDs
 * and pulse widths for the plunger and locking servos, as well as the IDs for the cage limit
 * switches.
 */
public final class ClimberRevServoRealConstants {
  /** The channel ID for the plunger servo */
  public static final ServoChannel.ChannelId PLUNGER_SERVO_ID = ServoChannel.ChannelId.kChannelId2;

  /** The channel ID for the locking servo */
  public static final ServoChannel.ChannelId LOCKING_SERVO_ID = ServoChannel.ChannelId.kChannelId0;

  /** The channel ID for the locking servo left */
  public static final ServoChannel.ChannelId ELEVATOR_LOCKING_SERVO_ID_LEFT =
      ServoChannel.ChannelId.kChannelId4;

  /** The channel ID for the locking servo right */
  public static final ServoChannel.ChannelId ELEVATOR_LOCKING_SERVO_ID_RIGHT =
      ServoChannel.ChannelId.kChannelId5;

  /** The ID for the first cage l Limit switch */
  public static final int CAGE_SWITCH_1_ID = 0;

  /** The ID for the second cage limit switch */
  public static final int CAGE_SWITCH_2_ID = 1;

  /** The pulse width for the plunger servo at 0 degrees DOWN */
  public static final int PLUNGER_SERVO_0_PW = 680;

  /** The pulse width for the plunger servo at 180 degrees */
  public static final int PLUNGER_SERVO_180_PW = 2500;

  /** The pulse width for the locking servo in the open position */
  public static final int LOCKING_SERVO_OPEN_PW = 1994;

  /** The pulse width for the locking servo in the closed position */
  public static final int LOCKING_SERVO_CLOSED_PW = 1376;

  public static final int ELEVATOR_LOCKING_LEFT_SERVO_OPEN_PW = 1600;

  public static final int ELEVATOR_LOCKING_LEFT_SERVO_CLOSED_PW = 1050;

  public static final int ELEVATOR_LOCKING_RIGHT_SERVO_OPEN_PW = 1470;

  public static final int ELEVATOR_LOCKING_RIGHT_SERVO_CLOSED_PW = 2050;
}
