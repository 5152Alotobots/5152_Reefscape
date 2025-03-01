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
package frc.alotobots.reefscape.subsystems.algaeintake.io;

import static edu.wpi.first.units.Units.Amps;
import static frc.alotobots.Constants.CanId.*;
import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeTalonFXRealConstants.*;
import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeTalonFXRealConstants.MotorSafetyLimits.STATOR_AMP_LIMIT;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeTalonFXRealConstants;

/**
 * Hardware implementation of the AlgaeIntake subsystem using TalonFX motors and a CANRange sensor.
 * This class manages two TalonFX motors (left and right) and a CANRange for intake feedback. The
 * right motor follows the left motor in an inverted configuration to ensure synchronized movement.
 */
public class AlgaeIntakeIOTalonFXReal implements AlgaeIntakeIO {

  /** The primary TalonFX motor controller for the algae intake */
  private final TalonFX leftTalon;

  /** The follower TalonFX motor controller for the algae intake */
  private final TalonFX rightTalon;

  /** The CANRange sensor */
  private final CANrange canRange;

  /** Velocity voltage control request for standard velocity-based control */
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

  /** Status signal for the current PID slot */
  private final StatusSignal<Integer> currentPidSlot;

  /** Status signal for the left motor's applied voltage */
  private final StatusSignal<Voltage> leftAppliedVoltage;

  /** Status signal for the left motor's applied current */
  private final StatusSignal<Current> leftAppliedCurrent;

  /** Status signal for the left motor's velocity */
  private final StatusSignal<AngularVelocity> leftVelocity;

  /** Status signal for the left motor's acceleration */
  private final StatusSignal<AngularAcceleration> leftAcceleration;

  /** Status signal for the left motor's position */
  private final StatusSignal<Angle> leftPosition;

  /** Status signal for the right motor's applied voltage */
  private final StatusSignal<Voltage> rightAppliedVoltage;

  /** Status signal for the right motor's applied current */
  private final StatusSignal<Current> rightAppliedCurrent;

  /** Status signal for the right motor's velocity */
  private final StatusSignal<AngularVelocity> rightVelocity;

  /** Status signal for the right motor's acceleration */
  private final StatusSignal<AngularAcceleration> rightAcceleration;

  /** Status signal for the right motor's position */
  private final StatusSignal<Angle> rightPosition;

  /** Status signal for the CANRange's proximity sensor */
  private final StatusSignal<Boolean> canRangeInProximity;

  /** Debouncer for filtering left motor connection status */
  private final Debouncer leftConnectedDebounce = new Debouncer(0.5);

  /** Debouncer for filtering right motor connection status */
  private final Debouncer rightConnectedDebounce = new Debouncer(0.5);

  /** Debouncer for filtering CANrange connection status */
  private final Debouncer canRangeConnectedDebounce = new Debouncer(0.5);

  /**
   * Constructs a new AlgaeIntakeIOTalonFXReal instance. Initializes and configures the TalonFX
   * motors and CANRange sensor with appropriate settings for position control, current limits, and
   * safety features. The right motor is configured to follow the left motor in an inverted
   * configuration.
   */
  public AlgaeIntakeIOTalonFXReal() {
    leftTalon = new TalonFX(ALGAE_INTAKE_LEFT_MOTOR_CAN_ID);
    rightTalon = new TalonFX(ALGAE_INTAKE_RIGHT_MOTOR_CAN_ID);
    canRange = new CANrange(ALGAE_INTAKE_CANRANGE_ID);

    // Left motor config
    var leftConfig = new TalonFXConfiguration();

    // PID configuration for velocity mode (Slot 0)
    leftConfig.Slot0.kP = AlgaeIntakeTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KP;
    leftConfig.Slot0.kI = AlgaeIntakeTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KI;
    leftConfig.Slot0.kD = AlgaeIntakeTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KD;
    leftConfig.Slot0.kA = AlgaeIntakeTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KA;
    leftConfig.Slot0.kG = AlgaeIntakeTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KG;
    leftConfig.Slot0.kS = AlgaeIntakeTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KS;
    leftConfig.Slot0.kV = AlgaeIntakeTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KV;

    leftConfig.MotorOutput.NeutralMode = MECHANISM_NEUTRAL_MODE;

    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    leftConfig.CurrentLimits.StatorCurrentLimit = STATOR_AMP_LIMIT.in(Amps);
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true

    leftConfig.MotorOutput.Inverted = LEFT_MOTOR_DIRECTION;

    // Apply config to left motor
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftConfig, 0.25));

    // Set right motor to be inverted follower
    tryUntilOk(
        5,
        () -> rightTalon.setControl(new Follower(leftTalon.getDeviceID(), RIGHT_MOTOR_INVERTED)));

    // CANRange config
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;
    canRangeConfig.ToFParams.UpdateFrequency = 50; // Hz
    canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    canRangeConfig.ProximityParams.ProximityHysteresis = .001;
    canRangeConfig.ProximityParams.ProximityThreshold = .15;
    canRangeConfig.FovParams.FOVRangeX = 7.5;
    canRangeConfig.FovParams.FOVRangeY = 7.5;

    // Apply config to CANRange
    tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig, 0.25));

    currentPidSlot = leftTalon.getClosedLoopSlot();

    leftPosition = leftTalon.getPosition();
    rightPosition = rightTalon.getPosition();

    leftVelocity = leftTalon.getVelocity();
    rightVelocity = rightTalon.getVelocity();

    leftAcceleration = leftTalon.getAcceleration();
    rightAcceleration = rightTalon.getAcceleration();

    leftAppliedVoltage = leftTalon.getMotorVoltage();
    rightAppliedVoltage = rightTalon.getMotorVoltage();

    leftAppliedCurrent = leftTalon.getStatorCurrent();
    rightAppliedCurrent = rightTalon.getStatorCurrent();

    canRangeInProximity = canRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        rightPosition,
        leftVelocity,
        rightVelocity,
        leftAcceleration,
        rightAcceleration,
        leftAppliedVoltage,
        rightAppliedVoltage,
        leftAppliedCurrent,
        rightAppliedCurrent,
        currentPidSlot,
        canRangeInProximity);
    ParentDevice.optimizeBusUtilizationForAll(leftTalon, rightTalon, canRange);
  }

  /**
   * Updates the input values for the algae intake subsystem. Refreshes all status signals and
   * updates the provided inputs object with current sensor readings and state information.
   *
   * @param inputs The AlgaeIntakeIOInputs object to update with current values
   */
  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    var leftSignals =
        BaseStatusSignal.refreshAll(
            leftPosition, leftVelocity, leftAcceleration, leftAppliedVoltage, leftAppliedCurrent);
    var rightSignals =
        BaseStatusSignal.refreshAll(
            rightPosition,
            rightVelocity,
            rightAcceleration,
            rightAppliedVoltage,
            rightAppliedCurrent);
    var canRangeSignals = BaseStatusSignal.refreshAll(canRangeInProximity);

    // Current slot
    inputs.currentPidSlot = currentPidSlot.getValue();

    // Connected status
    inputs.leftMotorConnected = leftConnectedDebounce.calculate(leftSignals.isOK());
    inputs.rightMotorConnected = rightConnectedDebounce.calculate(rightSignals.isOK());
    inputs.canRangeConnected = canRangeConnectedDebounce.calculate(canRangeSignals.isOK());

    // Velocities
    inputs.leftVelocity = leftVelocity.getValue();
    inputs.rightVelocity = rightVelocity.getValue();

    // Acceleration
    inputs.leftAcceleration = leftAcceleration.getValue();
    inputs.rightAcceleration = rightAcceleration.getValue();

    // Volts
    inputs.leftAppliedVolts = leftAppliedVoltage.getValue();
    inputs.rightAppliedVolts = rightAppliedVoltage.getValue();

    // Amps
    inputs.leftCurrentAmps = leftAppliedCurrent.getValue();
    inputs.rightCurrentAmps = rightAppliedCurrent.getValue();

    inputs.canRangeInProximity = canRangeInProximity.getValue();
  }

  /**
   * Sets the AlgaeIntake to a specific velocity using closed-loop control.
   *
   * @param velocity The target velocity as a AngularVelocity unit
   * @param pidSlot The PID slot to use (0 for velocity mode, 1 for position mode)
   */
  @Override
  public void setAlgaeIntakeVelocity(AngularVelocity velocity, int pidSlot) {
    leftTalon.setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot));
  }

  /**
   * Sets the algae intake motors to run in open-loop mode at the specified output percentage.
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  @Override
  public void setAlgaeIntakeOpenLoop(double percentOutput) {
    leftTalon.set(percentOutput);
  }

  /**
   * Sets the brake mode for the algae intake motors.
   *
   * @param brake true to enable brake mode, false for coast mode
   */
  @Override
  public void setAlgaeIntakeBrakeMode(boolean brake) {
    leftTalon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /** Stops all algae intake motor movement. */
  @Override
  public void stop() {
    leftTalon.stopMotor();
  }
}
