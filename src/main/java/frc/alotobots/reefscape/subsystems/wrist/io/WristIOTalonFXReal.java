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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.alotobots.Constants.CanId.WRIST_ENCODER_CAN_ID;
import static frc.alotobots.Constants.CanId.WRIST_MOTOR_CAN_ID;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXRealConstants.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXRealConstants.MotorSafetyLimits.*;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXRealConstants;

/**
 * Hardware implementation of the WristIO interface using TalonFX motor controller and CANCoder for
 * real robot operation. This class manages the physical wrist mechanism, handling motor control,
 * position sensing, and safety limits.
 */
public class WristIOTalonFXReal implements WristIO {

  /** Main TalonFX motor controller for the wrist mechanism */
  private final TalonFX wristTalon;

  /** CANCoder encoder for precise wrist position measurement */
  private final CANcoder wristEncoder;

  /** Control mode for position control using torque-based Field-Oriented Control */
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);

  /** Control mode for position control using direct voltage */
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  private final MotionMagicVoltage magicPositionVoltage = new MotionMagicVoltage(0);

  /** Control mode for velocity control using direct voltage */
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  // Status Signals for monitoring hardware state
  /** Current active PID slot being used for control */
  StatusSignal<Integer> currentPidSlot;

  /** Current voltage being applied to the motor */
  StatusSignal<Voltage> wristAppliedVoltage;

  /** Current being drawn by the wrist motor */
  StatusSignal<Current> wristAppliedCurrent;

  /** Current angular velocity of the wrist */
  StatusSignal<AngularVelocity> wristVelocity;

  StatusSignal<AngularAcceleration> wristAcceleration;

  /** Current angular position of the wrist */
  StatusSignal<Angle> wristPosition;

  /** Status of the forward/top software limit switch */
  StatusSignal<Boolean> topSoftLimit;

  /** Status of the reverse/bottom software limit switch */
  StatusSignal<Boolean> bottomSoftLimit;

  /** Debouncer to filter rapid changes in motor connection status */
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  /** Debouncer to filter rapid changes in encoder connection status */
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  /**
   * Creates a new WristIOTalonFXReal instance and configures all motor controller and encoder
   * settings. This includes PID configurations, software limits, current limits, and sensor
   * settings.
   */
  public WristIOTalonFXReal() {
    wristTalon = new TalonFX(WRIST_MOTOR_CAN_ID);
    wristEncoder = new CANcoder(WRIST_ENCODER_CAN_ID);

    var wristMotorConfig = new TalonFXConfiguration();

    // PID configuration for velocity mode (Slot 0)
    wristMotorConfig.Slot0.kP = WristTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KP;
    wristMotorConfig.Slot0.kI = WristTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KI;
    wristMotorConfig.Slot0.kD = WristTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KD;
    wristMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    wristMotorConfig.Slot0.kA = WristTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KA;
    wristMotorConfig.Slot0.kG = WristTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KG;
    wristMotorConfig.Slot0.kS = WristTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KS;
    wristMotorConfig.Slot0.kV = WristTalonFXRealConstants.PIDConstants.VelocityPIDConstants.KV;

    // PID configuration for position mode (Slot 1)
    wristMotorConfig.Slot1.kP = WristTalonFXRealConstants.PIDConstants.PositionPIDConstants.KP;
    wristMotorConfig.Slot1.kI = WristTalonFXRealConstants.PIDConstants.PositionPIDConstants.KI;
    wristMotorConfig.Slot1.kD = WristTalonFXRealConstants.PIDConstants.PositionPIDConstants.KD;
    wristMotorConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    wristMotorConfig.Slot1.kA = WristTalonFXRealConstants.PIDConstants.PositionPIDConstants.KA;
    wristMotorConfig.Slot1.kG = WristTalonFXRealConstants.PIDConstants.PositionPIDConstants.KG;
    wristMotorConfig.Slot1.kS = WristTalonFXRealConstants.PIDConstants.PositionPIDConstants.KS;
    wristMotorConfig.Slot1.kV = WristTalonFXRealConstants.PIDConstants.PositionPIDConstants.KV;

    wristMotorConfig.MotorOutput.NeutralMode = WristTalonFXRealConstants.MECHANISM_NEUTRAL_MODE;

    wristMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = LIMITS_ENABLED;
    wristMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = LIMITS_ENABLED;
    wristMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);
    wristMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);

    wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristMotorConfig.Feedback.FeedbackRemoteSensorID = WRIST_ENCODER_CAN_ID;
    wristMotorConfig.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;

    wristMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    wristMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = TORQUE_REVERSE_AMP_LIMIT.in(Amps);

    wristMotorConfig.CurrentLimits.StatorCurrentLimit = STATOR_AMP_LIMIT.in(Amps);
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    wristMotorConfig.MotorOutput.Inverted = MOTOR_DIRECTION;

    wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        MotionMagicConstants.CRUSE_VELOCITY.in(RotationsPerSecond);
    wristMotorConfig.MotionMagic.MotionMagicAcceleration =
        MotionMagicConstants.ACCELERATION.in(RotationsPerSecondPerSecond);
    wristMotorConfig.MotionMagic.MotionMagicJerk = MotionMagicConstants.JERK;

    tryUntilOk(5, () -> wristTalon.getConfigurator().apply(wristMotorConfig, 0.25));

    var wristEncoderConfig = new CANcoderConfiguration();

    wristEncoderConfig.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET;
    wristEncoderConfig.MagnetSensor.SensorDirection = ENCODER_DIRECTION;

    tryUntilOk(5, () -> wristEncoder.getConfigurator().apply(wristEncoderConfig, 0.25));

    // Initialize status signals
    currentPidSlot = wristTalon.getClosedLoopSlot();
    wristAppliedVoltage = wristTalon.getMotorVoltage();
    wristAppliedCurrent = wristTalon.getStatorCurrent();
    wristVelocity = wristTalon.getVelocity();
    wristAcceleration = wristTalon.getAcceleration();
    wristPosition = wristTalon.getPosition();
    topSoftLimit = wristTalon.getFault_ForwardSoftLimit();
    bottomSoftLimit = wristTalon.getFault_ReverseSoftLimit();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        currentPidSlot,
        wristAppliedVoltage,
        wristAppliedCurrent,
        wristVelocity,
        wristAcceleration,
        wristPosition,
        topSoftLimit,
        bottomSoftLimit);

    ParentDevice.optimizeBusUtilizationForAll(wristTalon);
  }

  /**
   * Updates the input values for the wrist subsystem by reading the latest status from hardware.
   * This includes position, velocity, current, voltage, and limit switch states.
   *
   * @param inputs The WristIOInputs object to update with the latest hardware state
   */
  @Override
  public void updateInputs(WristIOInputs inputs) {
    var wristSignals =
        BaseStatusSignal.refreshAll(
            currentPidSlot,
            wristAppliedVoltage,
            wristAppliedCurrent,
            wristVelocity,
            wristPosition,
            topSoftLimit,
            bottomSoftLimit);

    // Update connection status
    inputs.motorConnected = motorConnectedDebouncer.calculate(wristSignals.isOK());
    inputs.cancoderConnected = encoderConnectedDebouncer.calculate(wristEncoder.isConnected());

    // Update all input values
    inputs.pidSlot = currentPidSlot.getValue();
    inputs.motorAppliedVolts = wristAppliedVoltage.getValue();
    inputs.motorCurrent = wristAppliedCurrent.getValue();
    inputs.rotationVelocity = wristVelocity.getValue();
    inputs.mechanismAngle = wristPosition.getValue();
    inputs.topLimit = topSoftLimit.getValue();
    inputs.bottomLimit = bottomSoftLimit.getValue();
  }

  /**
   * Sets the wrist to a target position using closed-loop control.
   *
   * @param rotation The target angle to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  @Override
  public void setWristPosition(Angle rotation, int pidSlot) {
    wristTalon.setControl(positionVoltage.withPosition(rotation).withSlot(pidSlot));
  }

  /**
   * Sets the wrist to a target position using closed-loop control & motion magic.
   *
   * @param rotation The target angle to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  @Override
  public void setWristPositionMotionMagic(Angle rotation, int pidSlot) {
    wristTalon.setControl(positionVoltage.withPosition(rotation).withSlot(pidSlot));
  }

  /**
   * Sets the wrist to run at a target velocity using closed-loop control.
   *
   * @param velocity The target velocity to move at
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  @Override
  public void setWristVelocity(AngularVelocity velocity, int pidSlot) {
    wristTalon.setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot));
  }

  /**
   * Sets the wrist motor to run in open-loop mode at a specified percentage of maximum output.
   *
   * @param percentOutput The motor output percentage (-1.0 to 1.0)
   */
  @Override
  public void setWristOpenLoop(double percentOutput) {
    wristTalon.set(percentOutput);
  }

  /** Stops the wrist motor by setting the motor output to zero. */
  @Override
  public void stop() {
    wristTalon.stopMotor();
  }
}
