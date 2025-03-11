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
package frc.alotobots.library.subsystems.vision.localizationfusion;

import lombok.Getter;

@Getter
public enum LocalizationSourceState {
  FAULT(1),
  DISABLED(2),
  INITIALIZING(3),
  DEGRADED(4),
  READY(5);

  private final int priority;

  LocalizationSourceState(int priority) {
    this.priority = priority;
  }
}
