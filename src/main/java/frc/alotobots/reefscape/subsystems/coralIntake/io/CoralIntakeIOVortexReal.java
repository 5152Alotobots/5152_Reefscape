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
package frc.alotobots.reefscape.subsystems.coralIntake.io;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.Constants.CanId.INTAKE_CANRANGE_ID;
import static frc.alotobots.Constants.CanId.INTAKE_MOTOR_CAN_ID;
import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeVortexRealConstants.MECHANISM_NEUTRAL_MODE;
import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeVortexRealConstants.MOTOR_DIRECTION;
import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeVortexRealConstants.MotorSafetyLimits.STATOR_AMP_LIMIT;
import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeVortexRealConstants.PIDConstants.VelocityPIDConstants.*;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;

public class CoralIntakeIOVortexReal implements CoralIntakeIO {

  private final SparkFlex intakeMotor;
  private final CANrange canRange;

  private final StatusSignal<Boolean> intakeOccupied;

  // Yay! RevLib doesn't do status signals! I Hate Rev!

  /** Debouncer for filtering CANrange connection status */
  private final Debouncer canRangeConnectedDebounce = new Debouncer(0.5);

  /** Debouncer for filtering motor connection status */
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  public CoralIntakeIOVortexReal() {
    intakeMotor = new SparkFlex(INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    canRange = new CANrange(INTAKE_CANRANGE_ID);

    var motorConfig = new SparkFlexConfig();

    motorConfig.closedLoop.pidf(KP, KI, KD, KF);
    motorConfig.smartCurrentLimit(((int) STATOR_AMP_LIMIT.in(Amps)));
    motorConfig.signals.primaryEncoderVelocityAlwaysOn(true);
    motorConfig.signals.primaryEncoderPositionAlwaysOn(true);
    motorConfig.signals.primaryEncoderPositionPeriodMs(20);
    motorConfig.signals.primaryEncoderVelocityPeriodMs(20);

    motorConfig.idleMode(MECHANISM_NEUTRAL_MODE);

    motorConfig.inverted(MOTOR_DIRECTION);

    intakeMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var canRangeConfig = new CANrangeConfiguration();

    canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;
    canRangeConfig.ToFParams.UpdateFrequency = 100; // Hz
    canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 5000;
    canRangeConfig.ProximityParams.ProximityHysteresis = .03;
    canRangeConfig.ProximityParams.ProximityThreshold = .1;
    canRangeConfig.FovParams.FOVRangeX = 27;
    canRangeConfig.FovParams.FOVRangeY = 13;

    tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig, 0.25));

    intakeOccupied = canRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeOccupied);
    ParentDevice.optimizeBusUtilizationForAll(canRange);
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    var canRangeSignals = BaseStatusSignal.refreshAll(intakeOccupied);

    // Connected Status
    inputs.canRangeConnected = canRangeConnectedDebounce.calculate(canRangeSignals.isOK());
    inputs.motorConnected =
        motorConnectedDebounce.calculate(intakeMotor.getLastError().equals(REVLibError.kOk));

    // Positions
    inputs.intakeOccupied = intakeOccupied.getValue();

    // Velocities
    inputs.motorVelocity = RevolutionsPerSecond.of((intakeMotor.getEncoder().getVelocity()) / 60.0);

    // Volts
    inputs.motorAppliedVolts =
        Volts.of(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());

    // Amps
    inputs.motorCurrentAmps = Amps.of(intakeMotor.getOutputCurrent());
  }

  @Override
  public void setIntakeOpenLoop(double percent) {
    intakeMotor.set(percent);
  }

  @Override
  public void setIntakeVelocity(AngularVelocity velocity, int pidSlot) {
    throw new UnsupportedOperationException(
        "setIntakeVelocity of CoralIntakeIOVortexReal is not implemented yet");
  }

  @Override
  public boolean getIntakeOccupied() {
    return intakeOccupied.getValue();
  }

  @Override
  public void stop() {
    intakeMotor.set(0.0);
  }
}
