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
package frc.alotobots.util;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

import frc.alotobots.library.subsystems.bling.util.BlingDiagnosticManager;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public final class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    BlingDiagnosticManager.setPhoenixConfigStatus(BlingDiagnosticManager.ConfigStatus.IN_PROGRESS);

    StatusCode lastStatus = StatusCode.GeneralError;
    for (int i = 0; i < maxAttempts; i++) {
      lastStatus = command.get();
      if (lastStatus.isOK()) break;
    }

    if (lastStatus.isOK()) {
      BlingDiagnosticManager.setPhoenixConfigStatus(BlingDiagnosticManager.ConfigStatus.COMPLETE);
    } else {
      BlingDiagnosticManager.setPhoenixConfigStatus(BlingDiagnosticManager.ConfigStatus.ERROR);
    }

    return lastStatus;
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private static int instances = 0;
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX, boolean motorInverted) {
      this.id = instances++;

      this.talonFXSimState = talonFX.getSimState();
      talonFXSimState.Orientation =
          motorInverted
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;
    private final Angle encoderOffset;

    public TalonFXMotorControllerWithRemoteCancoderSim(
        TalonFX talonFX,
        boolean motorInverted,
        CANcoder cancoder,
        boolean encoderInverted,
        Angle encoderOffset) {
      super(talonFX, motorInverted);
      this.remoteCancoderSimState = cancoder.getSimState();
      this.remoteCancoderSimState.Orientation =
          encoderInverted
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
      this.encoderOffset = encoderOffset;
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle.minus(encoderOffset));
      remoteCancoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }
}
