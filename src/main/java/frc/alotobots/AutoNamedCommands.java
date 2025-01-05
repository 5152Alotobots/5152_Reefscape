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
package frc.alotobots;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import java.util.Map;
import lombok.experimental.UtilityClass;

/**
 * Utility class for registering named commands with PathPlanner's autonomous functionality. This
 * class manages a collection of commands that can be referenced by name in PathPlanner autonomous
 * routines.
 */
@UtilityClass
public class AutoNamedCommands {

  /**
   * Map containing all registered named commands. The key is the command name as a String, and the
   * value is the corresponding Command object.
   */
  private static final Map<String, Command> commands =
      new HashMap<>() {
        {
          // put("SOME_COMMAND_NAME", new Command() {});
        }
      };

  /**
   * Registers all predefined commands with PathPlanner's NamedCommands system. This method should
   * be called during swerve initialization to ensure all commands are available for autonomous
   * routines.
   */
  public static void setupNamedCommands() {
    NamedCommands.registerCommands(commands);
  }
}
