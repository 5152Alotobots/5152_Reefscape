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
package frc.alotobots.library.commands.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;

public class LogCommand extends InstantCommand {
  /**
   * Creates a new LogCommand that logs a value to AdvantageKit with a specified key
   *
   * @param key The key to log to
   * @param value The value to log
   */
  public LogCommand(String key, String value) {
    super(() -> Logger.recordOutput(key, value));
  }
}
