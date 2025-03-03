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
 * class manages three Xbox controllers:
 *
 * <ul>
 *   <li>Driver Controller: Primary robot movement and speed control
 *   <li>Co-Driver Controller: State-based controls and shared subsystem control
 *   <li>Co-Driver Backup Controller: Manual subsystem controls and redundant options
 * </ul>
 *
 * The class provides static methods to access controller inputs and defines button bindings for
 * commanding various robot subsystems and states.
 */
public class OI {
  /**
   * The minimum value that joystick inputs must exceed to be registered. This deadband prevents
   * unintended movement from controller drift and provides a stable neutral position for the
   * controls.
   */
  public static final double DEADBAND = 0.1;

  /** Controller port ID for the primary driver's Xbox controller. */
  private static final int DRIVER_CONTROLLER_ID = 0;

  /** Controller port ID for the co-driver's primary Xbox controller. */
  private static final int CO_DRIVER_CONTROLLER_ID = 1;

  /** Controller port ID for the co-driver's backup Xbox controller. */
  private static final int CO_DRIVER_BACKUP_CONTROLLER_ID = 2;

  /** Xbox controller instance for the primary driver's control functions. */
  private static final CommandXboxController driverController =
      new CommandXboxController(DRIVER_CONTROLLER_ID);

  /** Xbox controller instance for the co-driver's state-based and shared controls. */
  private static final CommandXboxController codriverController =
      new CommandXboxController(CO_DRIVER_CONTROLLER_ID);

  /** Xbox controller instance for manual and redundant control options. */
  private static final CommandXboxController codriverBackupController =
      new CommandXboxController(CO_DRIVER_BACKUP_CONTROLLER_ID);

  /**
   * Trigger that activates when the driver is using the chassis control sticks. Combines X/Y
   * translation and rotation inputs with deadband application to detect intentional driver input.
   */
  public static final Trigger hasDriverInput =
      new Trigger(
          () ->
              MathUtil.applyDeadband(driverController.getLeftX(), DEADBAND) != 0
                  || MathUtil.applyDeadband(driverController.getLeftY(), DEADBAND) != 0
                  || MathUtil.applyDeadband(driverController.getRightX(), DEADBAND) != 0);

  /**
   * Gets the forward/backward translation input from the driver's left stick.
   *
   * @return Value between -1.0 (backward) and 1.0 (forward)
   */
  public static double getTranslateForwardAxis() {
    return driverController.getLeftY();
  }

  /**
   * Gets the left/right translation input from the driver's left stick.
   *
   * @return Value between -1.0 (left) and 1.0 (right)
   */
  public static double getTranslateStrafeAxis() {
    return driverController.getLeftX();
  }

  /**
   * Gets the rotation input from the driver's right stick.
   *
   * @return Value between -1.0 (counter-clockwise) and 1.0 (clockwise)
   */
  public static double getRotationAxis() {
    return driverController.getRightX();
  }

  /**
   * Gets the turtle (slow) speed control input from the driver's left trigger. Used to enable
   * precise, slow movement for delicate operations.
   *
   * @return Value between 0.0 (not pressed) and 1.0 (fully pressed)
   */
  public static double getTurtleSpeedTrigger() {
    return driverController.getLeftTriggerAxis();
  }

  /**
   * Gets the turbo (fast) speed control input from the driver's right trigger. Used to enable
   * maximum speed movement for quick traversal.
   *
   * @return Value between 0.0 (not pressed) and 1.0 (fully pressed)
   */
  public static double getTurboSpeedTrigger() {
    return driverController.getRightTriggerAxis();
  }

  /* State-based play control triggers */
  public static final Trigger resetGyroButton =
      driverController.leftStick().and(driverController.rightStick());

  public static final Trigger stateCoralCoralStationButton = codriverController.rightBumper();
  public static final Trigger stateCoralStowedButton = codriverController.leftBumper();
  public static final Trigger stateCoralL4Button = codriverController.y();
  public static final Trigger stateCoralL3Button = codriverController.x();
  public static final Trigger stateCoralL2Button = codriverController.b();
  public static final Trigger stateCoralL1Button = codriverController.a();

  public static final Trigger coralIntakeReleaseButton = codriverController.rightTrigger();
  public static final Trigger algaeIntakeReleaseButton = codriverController.leftTrigger();

  public static final Trigger stateAlgaeL3L4Button = codriverController.povLeft();
  public static final Trigger stateAlgaeL2L3Button = codriverController.povRight();
  public static final Trigger stateAlgaeGroundButton = codriverController.povUp();
  public static final Trigger stateCoralGroundButton = codriverController.povDown();
  public static final Trigger stateAlgaeNetButton = codriverController.back();
  public static final Trigger stateAlgaeProcessorButton = codriverController.start();

  public static final Trigger climbButton = driverController.start();
  public static final Trigger unClimbButton = driverController.back();

  public static final Trigger coralIntakeIntakeManualButton = driverController.povDown();
  public static final Trigger coralIntakeEjectManualButton = driverController.povUp();
  // BACKUP -----------------------------------------------------------
  /* Backup control triggers */

  public static final Trigger ejectCoralButton = codriverBackupController.rightBumper();

  /** Trigger for activating the coral eject-through function */
  public static final Trigger coralIntakeEjectThroughButton = codriverBackupController.start();

  /* Wrist position control triggers */
  /** Trigger for moving the wrist to L4 coral position using backup D-pad up. */
  public static final Trigger wristL4coralButton = codriverBackupController.povUp();

  /** Trigger for moving the wrist to L2/L3 coral position using backup D-pad right. */
  public static final Trigger wristL2and3coralButton = codriverBackupController.povRight();

  /** Trigger for moving the wrist to ground position using backup D-pad left. */
  public static final Trigger wristGroundButton = codriverBackupController.povLeft();

  /* Elevator position control triggers */
  /** Trigger for moving the elevator to stow position using backup A button. */
  public static final Trigger elevatorStowButton = codriverBackupController.a();

  /** Trigger for moving the elevator to L2 position using backup B button. */
  public static final Trigger elevatorL2Button = codriverBackupController.b();

  /** Trigger for moving the elevator to L3 position using backup X button. */
  public static final Trigger elevatorL3Button = codriverBackupController.x();

  /** Trigger for moving the elevator to L4 position using backup Y button. */
  public static final Trigger elevatorL4Button = codriverBackupController.y();

  /**
   * Gets the manual elevator control input by selecting the larger magnitude input between the two
   * co-driver controllers. Applies deadband after selection.
   *
   * @return Value between -1.0 (down) and 1.0 (up)
   */
  public static double getElevatorAxis() {
    double primary = codriverController.getRightY();
    double backup = codriverBackupController.getRightY();
    return MathUtil.applyDeadband(
        Math.abs(primary) >= Math.abs(backup) ? primary : backup, DEADBAND);
  }

  /**
   * Gets the manual wrist control input by selecting the larger magnitude input between the two
   * co-driver controllers. Applies deadband after selection.
   *
   * @return Value between -1.0 and 1.0
   */
  public static double getWristAxis() {
    double primary = codriverController.getLeftY();
    double backup = codriverBackupController.getLeftY();
    return MathUtil.applyDeadband(
        Math.abs(primary) >= Math.abs(backup) ? primary : backup, DEADBAND);
  }

  /**
   * Groups axis-related constants together for better organization. Contains defined limits for
   * controller axis inputs.
   */
  public static final class AxisLimits {
    /**
     * Maximum value that a controller axis can output. Represents full forward/right on the stick.
     */
    public static final double MAX_AXIS_LIMIT = 1.0;

    /**
     * Minimum value that a controller axis can output. Represents full backward/left on the stick.
     */
    public static final double MIN_AXIS_LIMIT = -1.0;
  }
}
