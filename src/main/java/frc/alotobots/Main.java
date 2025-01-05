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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Main class serves as the entry point for the robot program. This class should not be modified
 * except to change the parameter class in the startRobot call. No static variables or
 * initialization should be added.
 */
public final class Main {
  /**
   * Private constructor to prevent instantiation. This class should not be instantiated as it only
   * contains a static main method.
   */
  private Main() {}

  /**
   * Main initialization method that starts the robot code. Do not perform any robot initialization
   * here. All robot initialization should be done in the Robot class.
   *
   * @param args Command line arguments (unused)
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
