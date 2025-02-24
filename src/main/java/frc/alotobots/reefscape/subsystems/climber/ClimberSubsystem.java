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
package frc.alotobots.reefscape.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.reefscape.subsystems.climber.io.ClimberIO;
import frc.alotobots.reefscape.subsystems.climber.io.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * A subsystem that controls the robot's climbing mechanism. This subsystem manages two servos: - A
 * plunger servo that can move between receive (180°) and plunge (0°) positions - A locking servo
 * that secures the climbing cage
 */
public class ClimberSubsystem extends SubsystemBase {
  /** The hardware interface for the climber */
  private final ClimberIO io;

  /** The logged inputs from the climber hardware */
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /**
   * Creates a new ClimberSubsystem.
   *
   * @param io The hardware interface for the climber
   */
  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  /** Periodic update function that logs climber inputs. Called once per scheduler run. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /**
   * Gets the state of the cage limit switches.
   *
   * @return true if the cage switches are activated
   */
  public boolean getCageSwitches() {
    return io.getCageSwitches();
  }

  /** Enables both the plunger and locking servos. */
  public void enableServos() {
    io.enablePlungerServo();
    io.enableLockingServo();
  }

  /** Disables both the plunger and locking servos. */
  public void disableServos() {
    io.disablePlungerServo();
    io.disableLockingServo();
  }

  /**
   * Sets the plunger servo to a specific angle. 0 degrees is the plunge position, 180 degrees is
   * the receive position.
   *
   * @param angle The desired angle for the plunger servo
   */
  public void setPlungerToAngle(Angle angle) {
    io.setPlungerServoPosition(angle);
  }

  /** Locks the climbing cage using the locking servo. */
  public void lockCage() {
    io.setLockingServoLocked(true);
  }

  /** Unlocks the climbing cage using the locking servo. */
  public void unlockCage() {
    io.setLockingServoLocked(false);
  }

  /** Enables only the plunger servo. */
  public void enablePlungerServo() {
    io.enablePlungerServo();
  }

  /** Enables only the locking servo. */
  public void enableLockingServo() {
    io.enableLockingServo();
  }

  /** Disables only the plunger servo. */
  public void disablePlungerServo() {
    io.disablePlungerServo();
  }

  /** Disables only the locking servo. */
  public void disableLockingServo() {
    io.disableLockingServo();
  }

  /** Sets the plunger servo to its receive position (180 degrees). */
  public void setPlungerToReceive() {
    io.setPlungerServoPosition(Degrees.of(180));
  }

  /** Sets the plunger servo to its plunge position (0 degrees). */
  public void setPlungerToPlunge() {
    io.setPlungerServoPosition(Degrees.of(0));
  }
}
