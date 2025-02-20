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
package frc.alotobots.util.commandbase;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

/**
 * A sequential command group that releases subsystem requirements between commands. This allows
 * default commands to run on subsystems after their commands complete.
 */
public class ReleasingSequentialCommandGroup extends Command {
  private final List<Command> commands;
  private int currentCommandIndex = -1;
  private Command currentCommand = null;

  public ReleasingSequentialCommandGroup(Command... commands) {
    this.commands = List.of(commands);
  }

  @Override
  public void initialize() {
    currentCommandIndex = -1;
    scheduleNextCommand();
  }

  @Override
  public void execute() {
    if (currentCommand == null || currentCommand.isFinished()) {
      scheduleNextCommand();
    }
  }

  private void scheduleNextCommand() {
    // End the current command if it exists
    if (currentCommand != null) {
      currentCommand.end(false);
    }

    currentCommandIndex++;
    if (currentCommandIndex < commands.size()) {
      currentCommand = commands.get(currentCommandIndex);
      currentCommand.initialize();
    } else {
      currentCommand = null;
    }
  }

  @Override
  public boolean isFinished() {
    return currentCommandIndex >= commands.size();
  }

  @Override
  public void end(boolean interrupted) {
    if (currentCommand != null) {
      currentCommand.end(interrupted);
    }
  }

  /**
   * Adds one or more commands to the group.
   *
   * @param commands Commands to add to the group
   */
  public void addCommands(Command... commands) {
    requireNonNull(commands, "commands");

    if (currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a sequential command group while the group is running");
    }

    this.commands.addAll(List.of(commands));
  }

  private void requireNonNull(Command[] commands, String message) {
    if (commands == null) {
      throw new NullPointerException(message);
    }
    for (Command command : commands) {
      if (command == null) {
        throw new NullPointerException(message);
      }
    }
  }
}
