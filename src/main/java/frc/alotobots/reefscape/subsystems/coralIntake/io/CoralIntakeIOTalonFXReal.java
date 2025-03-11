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

import static edu.wpi.first.units.Units.Amps;
import static frc.alotobots.Constants.CanId.INTAKE_CANRANGE_ID;
import static frc.alotobots.Constants.CanId.INTAKE_MOTOR_CAN_ID;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeTalonFXRealConstants;
import frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeTalonFXRealConstants.MotorSafetyLimits;

/**
 * Hardware implementation of the CoralIntakeIO interface for REV Robotics SparkFlex motor
 * controllers and CANrange sensors. This implementation provides real hardware control and sensor
 * feedback for the coral intake mechanism.
 */
public class CoralIntakeIOTalonFXReal implements CoralIntakeIO {
  /** Main TalonFX motor controller for the wrist mechanism */
  private final TalonFX intakeMotor;

  private final CANrange canRange;

  /** Control mode for velocity control using direct voltage */
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  /** Control mode for open loop output */
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  // Status Signals for monitoring hardware state
  StatusSignal<Integer> currentPidSlot;
  StatusSignal<Voltage> intakeAppliedVoltage;
  StatusSignal<Current> intakeAppliedCurrent;
  StatusSignal<AngularVelocity> intakeVelocity;
  StatusSignal<AngularAcceleration> intakeAcceleration;
  StatusSignal<Angle> intakePosition;

  StatusSignal<Boolean> intakeOccupied;

  /** Debounce for motor connection status */
  private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

  private final Debouncer canRangeConnectedDebounce = new Debouncer(0.5);

  /**
   * Creates a new CoralIntakeIOVortexReal instance and configures all hardware devices. Sets up
   * motor controller parameters, CANrange sensor configuration, and status signals.
   */
  public CoralIntakeIOTalonFXReal() {
    intakeMotor = new TalonFX(INTAKE_MOTOR_CAN_ID);
    canRange = new CANrange(INTAKE_CANRANGE_ID);

    var intakeMotorConfig = new TalonFXConfiguration();

    currentPidSlot = intakeMotor.getClosedLoopSlot();
    intakeAppliedVoltage = intakeMotor.getMotorVoltage();
    intakeAppliedCurrent = intakeMotor.getStatorCurrent();
    intakeVelocity = intakeMotor.getVelocity();
    intakeAcceleration = intakeMotor.getAcceleration();
    intakePosition = intakeMotor.getPosition();

    intakeMotorConfig.MotorOutput.NeutralMode =
        CoralIntakeTalonFXRealConstants.MECHANISM_NEUTRAL_MODE;

    intakeMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        MotorSafetyLimits.TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    intakeMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        MotorSafetyLimits.TORQUE_REVERSE_AMP_LIMIT.in(Amps);

    intakeMotorConfig.CurrentLimits.StatorCurrentLimit =
        MotorSafetyLimits.STATOR_AMP_LIMIT.in(Amps);
    intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeMotorConfig.MotorOutput.Inverted = CoralIntakeTalonFXRealConstants.MOTOR_DIRECTION;

    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeMotorConfig, 0.25));

    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;
    canRangeConfig.ToFParams.UpdateFrequency = 50; // Hz
    canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 5000;
    canRangeConfig.ProximityParams.ProximityHysteresis = .03;
    canRangeConfig.ProximityParams.ProximityThreshold = .13;
    canRangeConfig.FovParams.FOVRangeX = 27;
    canRangeConfig.FovParams.FOVRangeY = 13;

    tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig, 0.25));

    intakeOccupied = canRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeOccupied);
    ParentDevice.optimizeBusUtilizationForAll(canRange);
  }

  /**
   * Updates all input values with the latest hardware state.
   *
   * @param inputs The input object to update with new values
   */
  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    var canRangeSignals = BaseStatusSignal.refreshAll(intakeOccupied);

    // Connected Status
    inputs.intakeOccupied = intakeOccupied.getValue();
    inputs.canRangeConnected = canRangeConnectedDebounce.calculate(canRangeSignals.isOK());
    inputs.motorConnected = motorConnectedDebounce.calculate(intakeMotor.isConnected());
    // Positions
    inputs.intakeOccupied = intakeOccupied.getValue();

    // Velocities
    inputs.motorVelocity = intakeVelocity.getValue();

    // Volts
    inputs.motorAppliedVolts = intakeAppliedVoltage.getValue();
    // Amps
    inputs.motorCurrentAmps = intakeAppliedCurrent.getValue();
  }

  /**
   * Sets the intake motor to run at a specified percentage of full power.
   *
   * @param percent Motor output percentage (-1.0 to 1.0)
   */
  @Override
  public void setIntakeOpenLoop(double percent) {
    intakeMotor.set(percent);
  }

  /**
   * Sets the intake motor to run at a specified velocity using closed-loop control. [NOT YET
   * IMPLEMENTED]
   *
   * @param velocity Target velocity for the intake
   * @param pidSlot PID slot to use for velocity control
   * @throws UnsupportedOperationException This feature is not yet implemented
   */
  @Override
  public void setIntakeVelocity(AngularVelocity velocity, int pidSlot) {
    throw new UnsupportedOperationException(
        "setIntakeVelocity of CoralIntakeIOVortexReal is not implemented yet");
  }

  /**
   * Gets the current game piece detection status from the CANrange sensor.
   *
   * @return true if a game piece is detected in the intake
   */
  @Override
  public boolean getIntakeOccupied() {
    return intakeOccupied.getValue();
  }

  /** Stops all intake motor movement. */
  @Override
  public void stop() {
    intakeMotor.set(0.0);
  }
}
