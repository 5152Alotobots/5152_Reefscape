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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOTalonFXSim implements WristIO {
  private final SingleJointedArmSim wristSim;

  private final DCMotor motor = DCMotor.getFalcon500(1);

  private double appliedVolts = 0.0;
  private int currentPidSlot = 0;
  private boolean brakeMode = true;

  public WristIOTalonFXSim() {
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
    inputs.pidSlot = currentPidSlot;
    inputs.motorCurrent = Amps.of(wristSim.getCurrentDrawAmps());
  }

  @Override
  public void setWristOpenLoop(double percentOutput) {
    appliedVolts = percentOutput * 12.0;
    wristSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    wristSim.setInputVoltage(0.0);
  }
}
