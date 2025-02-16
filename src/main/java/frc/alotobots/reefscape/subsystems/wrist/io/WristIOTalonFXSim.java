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
package frc.alotobots.reefscape.subsystems.wrist.io;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristIOTalonFXConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class WristIOTalonFXSim implements WristIO {
  private final SingleJointedArmSim wristSim;
  @AutoLogOutput private final LoggedMechanism2d wristMech = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismLigament2d wristArm =
      new LoggedMechanismLigament2d("wrist", 0.5, 180, 6, new Color8Bit(Color.kPurple));

  private final DCMotor motor = DCMotor.getFalcon500(1);

  private double appliedVolts = 0.0;
  private int currentPidSlot = 0;
  private boolean brakeMode = true;

  public WristIOTalonFXSim() {
    wristMech.getRoot("wrist", 1.5, 1.5).append(wristArm);

    wristSim =
        new SingleJointedArmSim(
            motor,
            ROTOR_TO_SENSOR_RATIO,
            INERTIA_KGMETERSSQURD,
            ARM_LENGTH,
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            MIN_ANGLE.in(Radians));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    wristSim.update(0.2);

    inputs.motorConnected = true;
    inputs.topLimit = wristSim.getAngleRads() >= MIN_ANGLE.in(Radians);
    inputs.bottomLimit = wristSim.getAngleRads() <= MAX_ANGLE.in(Radians);

    inputs.rotationVelocity = RadiansPerSecond.of(wristSim.getVelocityRadPerSec());

    inputs.motorAppliedVolts = Volts.of(appliedVolts);

    inputs.position = Radians.of(wristSim.getAngleRads());
    wristArm.setAngle(Rotation2d.fromDegrees(inputs.position.in(Degree)));
    inputs.pidSlot = currentPidSlot;
    inputs.motorCurrent = Amps.of(wristSim.getCurrentDrawAmps());
  }

  @Override
  public void setWristOpenLoop(double percentOutput) {
    appliedVolts = percentOutput * 13.0;
    wristSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    wristSim.setInputVoltage(0.0);
  }
}
