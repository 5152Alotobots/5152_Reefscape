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

import edu.wpi.first.math.MathUtil;
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

  /** Driver controller ID */
  private static final int DRIVER_CONTROLLER_ID = 0;

  /** Co-Driver controller ID */
  private static final int CO_DRIVER_CONTROLLER_ID = 1;

  /** The primary driver's controller. Used for main robot control functions. */
  private static final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_ID);

  private static final CommandXboxController codriverController =
      new CommandXboxController(CO_DRIVER_CONTROLLER_ID);

  /** Trigger for when the driver is using the controller sticks to control the chassis */
  public static final Trigger hasDriverInput =
      new Trigger(
          () ->
              MathUtil.applyDeadband(driverController.getLeftX(), DEADBAND) != 0
                  || MathUtil.applyDeadband(driverController.getLeftY(), DEADBAND) != 0
                  || MathUtil.applyDeadband(driverController.getRightX(), DEADBAND) != 0);

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

  /*Intake Buttons */
  public static Trigger coralIntakeIntakeButton = codriverController.back();
  public static Trigger coralIntakeOuttakeThroughButton = codriverController.start();

  /* Wrist Position Buttons */
  public static Trigger wristL4coralButton = codriverController.povUp();
  public static Trigger wristL2and3coralButton = codriverController.povRight();
  public static Trigger wristGroundButton = codriverController.povLeft();

  /** Elevator Postition Buttons */
  public static Trigger elevatorStowButton = codriverController.a();

  public static Trigger elevatorL2Button = codriverController.b();
  public static Trigger elevatorL3Button = codriverController.x();
  public static Trigger elevatorL4Button = codriverController.y();

  /**
   * Gets the open loop elevator input from the co-driver controller
   *
   * @return Value between -1.0 (down) and 1.0 (up)
   */
  public static double getElevatorAxis() {
    return MathUtil.applyDeadband(codriverController.getRightY(), DEADBAND);
  }

  /**
   * Gets the wrist axis
   *
   * @return Value between -1.0 and 1.0
   */
  public static double getWristAxis() {
    return MathUtil.applyDeadband(codriverController.getLeftY(), DEADBAND);
  }
}
