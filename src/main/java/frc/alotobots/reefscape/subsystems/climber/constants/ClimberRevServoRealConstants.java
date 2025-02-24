package frc.alotobots.reefscape.subsystems.climber.constants;

import com.revrobotics.servohub.ServoChannel;

/**
 * Constants for the real servo configuration in the climber subsystem.
 * This class contains the IDs and pulse widths for the plunger and locking servos,
 * as well as the IDs for the cage limit switches.
 */
public final class ClimberRevServoRealConstants {
  /** The channel ID for the plunger servo */
  public static final ServoChannel.ChannelId PLUNGER_SERVO_ID = ServoChannel.ChannelId.kChannelId2;
  /** The channel ID for the locking servo */
  public static final ServoChannel.ChannelId LOCKING_SERVO_ID = ServoChannel.ChannelId.kChannelId0;

  /** The ID for the first cage limit switch */
  public static final int CAGE_SWITCH_1_ID = 0;
  /** The ID for the second cage limit switch */
  public static final int CAGE_SWITCH_2_ID = 1;

  /** The pulse width for the plunger servo at 0 degrees */
  public static final int PLUNGER_SERVO_0_PW = 650;
  /** The pulse width for the plunger servo at 180 degrees */
  public static final int PLUNGER_SERVO_180_PW = 2500;
  /** The pulse width for the locking servo in the open position */
  public static final int LOCKING_SERVO_OPEN_PW = 1773;
  /** The pulse width for the locking servo in the closed position */
  public static final int LOCKING_SERVO_CLOSED_PW = 1446;
}