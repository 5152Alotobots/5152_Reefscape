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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * A parallel command group that releases subsystem requirements as individual commands complete.
 * This allows default commands to run on subsystems as soon as their commands finish.
 */
public class ReleasingParallelCommandGroup extends Command {
  private final List<Command> commands;
  private final Set<Command> finishedCommands;

  public ReleasingParallelCommandGroup(Command... commands) {
    this.commands = List.of(commands);
    this.finishedCommands = new HashSet<>();
  }

  @Override
  public void initialize() {
    finishedCommands.clear();
    for (Command command : commands) {
      command.initialize();
    }
  }

  @Override
  public void execute() {
    for (Command command : commands) {
      if (!finishedCommands.contains(command)) {
        if (command.isFinished()) {
          finishedCommands.add(command);
          command.end(false);
        } else {
          command.execute();
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return finishedCommands.size() == commands.size();
  }

  @Override
  public void end(boolean interrupted) {
    for (Command command : commands) {
      if (!finishedCommands.contains(command)) {
        command.end(interrupted);
      }
    }
  }

  /**
   * Adds one or more commands to the group.
   *
   * @param commands Commands to add to the group
   */
  public void addCommands(Command... commands) {
    requireNonNull(commands, "commands");

    if (!finishedCommands.isEmpty()) {
      throw new IllegalStateException(
          "Commands cannot be added to a parallel command group while the group is running");
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
