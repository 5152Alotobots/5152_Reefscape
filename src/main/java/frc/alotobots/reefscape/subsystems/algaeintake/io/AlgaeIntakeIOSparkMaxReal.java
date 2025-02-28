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

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.Constants.CanId.ALGAE_INTAKE_CANRANGE_ID;
import static frc.alotobots.Constants.CanId.ALGAE_INTAKE_LEFT_MOTOR_CAN_ID;
import static frc.alotobots.Constants.CanId.ALGAE_INTAKE_RIGHT_MOTOR_CAN_ID;
import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeSparkMaxRealConstants.*;
import static frc.alotobots.reefscape.subsystems.algaeintake.constants.AlgaeIntakeSparkMaxRealConstants.MotorSafetyLimits.STATOR_AMP_LIMIT;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;

/**
 * Hardware implementation of the AlgaeIntakeIO interface for REV Robotics SparkMax motor
 * controllers with Neo 550 motors. This implementation provides real hardware control and sensor
 * feedback for the algae intake mechanism with two opposing motors.
 */

/**
 * Hardware implementation of the AlgaeIntakeIO interface for REV Robotics SparkMax motor
 * controllers with Neo 550 motors. This implementation provides real hardware control and sensor
 * feedback for the algae intake mechanism with two opposing motors.
 */
public class AlgaeIntakeIOSparkMaxReal implements AlgaeIntakeIO {

  /** The SparkMax motor controller for the left intake motor */
  private final SparkMax leftMotor;

  /** The SparkMax motor controller for the right intake motor */
  private final SparkMax rightMotor;

  /** The CANrange sensor for game piece detection */
  private final CANrange canRange;

  /** Status signal for game piece detection */
  private final StatusSignal<Boolean> intakeOccupied;

  /** Debouncer for filtering left motor connection status */
  private final Debouncer leftMotorConnectedDebounce = new Debouncer(0.5);

  /** Debouncer for filtering right motor connection status */
  private final Debouncer rightMotorConnectedDebounce = new Debouncer(0.5);

  /** Debouncer for filtering CANrange connection status */
  private final Debouncer canRangeConnectedDebounce = new Debouncer(0.5);

  /**
   * Creates a new AlgaeIntakeIOSparkMaxReal instance and configures all hardware devices. Sets up
   * motor controller parameters, sensors, and configures the Neo 550 motors for the algae intake.
   */
  public AlgaeIntakeIOSparkMaxReal() {
    leftMotor = new SparkMax(ALGAE_INTAKE_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ALGAE_INTAKE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    canRange = new CANrange(ALGAE_INTAKE_CANRANGE_ID);

    // Configure left motor
    var leftMotorConfig = new SparkMaxConfig();
    configureMotor(leftMotorConfig, LEFT_MOTOR_INVERTED);
    leftMotor.configure(
        leftMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Configure right motor
    var rightMotorConfig = new SparkMaxConfig();
    configureMotor(rightMotorConfig, RIGHT_MOTOR_INVERTED);
    rightMotor.configure(
        rightMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Configure CANrange sensor
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    canRangeConfig.ToFParams.UpdateFrequency = 50; // Hz
    canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    canRangeConfig.ProximityParams.ProximityHysteresis = .001;
    canRangeConfig.ProximityParams.ProximityThreshold = .15;
    canRangeConfig.FovParams.FOVRangeX = 7.5;
    canRangeConfig.FovParams.FOVRangeY = 7.5;

    tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig, 0.25));

    intakeOccupied = canRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeOccupied);
    ParentDevice.optimizeBusUtilizationForAll(canRange);
  }

  /**
   * Configures motor parameters for a SparkMax controller.
   *
   * @param config The configuration object to modify
   * @param inverted Whether the motor direction should be inverted
   */
  private void configureMotor(SparkMaxConfig config, boolean inverted) {
    // Configure basic parameters
    config.idleMode(MECHANISM_NEUTRAL_MODE);
    config.inverted(inverted);

    // Configure current limits
    config.smartCurrentLimit((int) STATOR_AMP_LIMIT.in(Amps));

    // Configure data logging
    config.signals.primaryEncoderVelocityAlwaysOn(true);
    config.signals.primaryEncoderPositionAlwaysOn(true);
    config.signals.primaryEncoderVelocityPeriodMs(20);
    config.signals.primaryEncoderPositionPeriodMs(20);
  }

  /**
   * Updates all input values with the latest hardware state.
   *
   * @param inputs The input object to update with new values
   */
  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    var canRangeSignals = BaseStatusSignal.refreshAll(intakeOccupied);

    // Connected Status
    inputs.leftMotorConnected =
        leftMotorConnectedDebounce.calculate(leftMotor.getLastError().equals(REVLibError.kOk));
    inputs.rightMotorConnected =
        rightMotorConnectedDebounce.calculate(rightMotor.getLastError().equals(REVLibError.kOk));
    inputs.canRangeConnected = canRangeConnectedDebounce.calculate(canRangeSignals.isOK());

    // Game piece detection from CANrange
    inputs.intakeOccupied = intakeOccupied.getValue();

    // Velocities - convert from RPM to rad/s
    inputs.leftMotorVelocity = RevolutionsPerSecond.of(leftMotor.getEncoder().getVelocity() / 60.0);
    inputs.rightMotorVelocity =
        RevolutionsPerSecond.of(rightMotor.getEncoder().getVelocity() / 60.0);

    // Volts
    inputs.leftMotorAppliedVolts =
        Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    inputs.rightMotorAppliedVolts =
        Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());

    // Amps
    inputs.leftMotorCurrentAmps = Amps.of(leftMotor.getOutputCurrent());
    inputs.rightMotorCurrentAmps = Amps.of(rightMotor.getOutputCurrent());

    // Temperature
    inputs.leftMotorTemp = Celsius.of(leftMotor.getMotorTemperature());
    inputs.rightMotorTemp = Celsius.of(rightMotor.getMotorTemperature());
  }

  /**
   * Sets the intake motors to run at a specified percentage of full power. The two motors run in
   * opposite directions to create the intake/outtake motion.
   *
   * @param percent Motor output percentage (-1.0 to 1.0)
   */
  @Override
  public void setIntakeOpenLoop(double percent) {
    // For intake (positive values):
    // - Left motor runs forward
    // - Right motor runs in opposite direction (handled by motor inversion)
    leftMotor.set(percent);
    rightMotor.set(percent);
  }

  /**
   * Gets the current game piece detection status from the digital sensor.
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
    leftMotor.set(0.0);
    rightMotor.set(0.0);
  }
}
