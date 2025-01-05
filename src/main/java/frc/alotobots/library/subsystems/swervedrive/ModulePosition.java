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
package frc.alotobots.library.subsystems.swervedrive;

public enum ModulePosition {
  FRONT_LEFT(0),
  FRONT_RIGHT(1),
  BACK_LEFT(2),
  BACK_RIGHT(3);

  public final int index;

  ModulePosition(int index) {
    this.index = index;
  }
}
