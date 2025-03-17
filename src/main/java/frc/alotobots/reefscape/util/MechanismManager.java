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
package frc.alotobots.reefscape.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class MechanismManager {

  // The 2D visualization canvas
  public static final LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);

  // Elevator root & ligament
  private static final LoggedMechanismRoot2d elevatorRoot = mechanism.getRoot("Elevator", 1, 0);
  private static final LoggedMechanismLigament2d elevatorLigament =
      elevatorRoot.append(
          new LoggedMechanismLigament2d(
              "Elevator",
              .23, // Start at minimum height
              90, // Vertical
              6,
              new Color8Bit(Color.kGreen)));

  // Wrist ligament (attached to the end of the elevator)
  private static final LoggedMechanismLigament2d wristLigament =
      elevatorLigament.append(
          new LoggedMechanismLigament2d(
              "Wrist",
              .3048,
              180, // Start horizontally
              6,
              new Color8Bit(Color.kYellow)));

  public static void logMech() {
    Logger.recordOutput("Mechanism/Mech", mechanism);
  }

  /**
   * Updates the elevator height.
   *
   * @param heightMeters The current height of the elevator.
   */
  public static void updateElevatorMech(Distance height) {
    elevatorLigament.setLength(height.in(Meters));
  }

  /**
   * Updates the wrist angle.
   *
   * @param wristAngleRads The current wrist angle in radians.
   */
  public static void updateWristMech(Angle wristAngle) {
    wristLigament.setAngle(wristAngle.in(Degrees) - 90); // Convert radians to degrees
  }
}
