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
package frc.alotobots;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The Operator Interface (OI) class handles all driver control inputs and button mappings. This
 * class provides methods to access controller inputs and defines button bindings for commanding the
 * robot.
 */
public class OI {
  /**
   * The minimum value that joystick inputs must exceed to be registered. Used to prevent drift and
   * unintended movement.
   */
  public static final double DEADBAND = 0.1;

  /** The primary driver's controller. Used for main robot control functions. */
  private static final CommandXboxController driverController = new CommandXboxController(0);

  /** Trigger for when the driver is using the controller sticks to control the chassis */
  public static final Trigger hasDriverInput =
      new Trigger(
          () ->
              Math.abs(driverController.getLeftX()) > DEADBAND
                  || Math.abs(driverController.getLeftY()) > DEADBAND
                  || Math.abs(driverController.getRightX()) > DEADBAND);

  /**
   * Gets the forward/backward translation input from the driver's controller.
   *
   * @return Value between -1.0 (backward) and 1.0 (forward)
   */
  public static double getTranslateForwardAxis() {
    return driverController.getLeftY();
  }

  /**
   * Gets the left/right translation input from the driver's controller.
   *
   * @return Value between -1.0 (left) and 1.0 (right)
   */
  public static double getTranslateStrafeAxis() {
    return driverController.getLeftX();
  }

  /**
   * Gets the rotation input from the driver's controller.
   *
   * @return Value between -1.0 (counter-clockwise) and 1.0 (clockwise)
   */
  public static double getRotationAxis() {
    return driverController.getRightX();
  }

  /**
   * Gets the turtle (slow) speed control input value.
   *
   * @return Value between 0.0 and 1.0
   */
  public static double getTurtleSpeedTrigger() {
    return driverController.getLeftTriggerAxis();
  }

  /**
   * Gets the turbo (fast) speed control input value.
   *
   * @return Value between 0.0 and 1.0
   */
  public static double getTurboSpeedTrigger() {
    return driverController.getRightTriggerAxis();
  }

  /** Enable pathfinding */
  public static Trigger enablePathfindingButton = driverController.back();

  /** Enable auto pathfinding */
  public static Trigger enableFullAutoPathfindingButton = driverController.start();

  /** Pathfind to the selected branch */
  public static Trigger pathfindToSelectedReefBranchButton = driverController.y();

  /** Pathfind to the selected coral station */
  public static Trigger pathfindToSelectedCoralStationButton = driverController.a();

  /** Cycles the selected pickup position one to the left */
  public static Trigger cycleCoralStationPickupPositionLeftButton = driverController.x();

  /** Cycles the selected pickup position one to the right */
  public static Trigger cycleCoralStationPickupPositionRightButton = driverController.b();

  /** Cycles the selected coral station one to the left */
  public static Trigger cycleCoralStationSideLeftButton = driverController.leftBumper();

  /** Cycles the selected coral station one to the right */
  public static Trigger cycleCoralStationSideRightButton = driverController.rightBumper();

  /** Cycles the selected branch one to the left */
  public static Trigger cycleSelectedBranchLeftButton = driverController.povLeft();

  /** Cycles the selected branch one to the right */
  public static Trigger cycleSelectedBranchRightButton = driverController.povRight();

  /** Cycles the branch level up once */
  public static Trigger cycleLevelUpButton = driverController.povUp();

  /** Cycles the branch level down once */
  public static Trigger cycleLevelDownButton = driverController.povDown();
}
