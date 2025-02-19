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
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXRealConstants;

public class WristIOTalonFXReal implements WristIO {

  // Motors
  private final TalonFX wristTalon;
  private final CANcoder wristEncoder;

  // Control Modes
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0);

  // Status Signals
  StatusSignal<Integer> currentPidSlot;

  StatusSignal<Voltage> wristAppliedVoltage;

  StatusSignal<Current> wristAppliedCurrent;

  StatusSignal<AngularVelocity> wristVelocity;

  StatusSignal<Angle> wristPosition;

  StatusSignal<Boolean> topSoftLimit;

  StatusSignal<Boolean> bottomSoftLimit;

  private final Debouncer wristConnectedDebouncer = new Debouncer(0.5);

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
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true

    wristMotorConfig.MotorOutput.Inverted = MOTOR_DIRECTION;

    tryUntilOk(5, () -> wristTalon.getConfigurator().apply(wristMotorConfig, 0.25));

    var wristEncoderConfig = new CANcoderConfiguration();

    wristEncoderConfig.MagnetSensor.MagnetOffset = ENCODER_MAGNET_OFFSET;
    wristEncoderConfig.MagnetSensor.SensorDirection = ENCODER_DIRECTION;

    // Apply config wrist
    tryUntilOk(5, () -> wristEncoder.getConfigurator().apply(wristEncoderConfig, 0.25));

    currentPidSlot = wristTalon.getClosedLoopSlot();
    wristAppliedVoltage = wristTalon.getMotorVoltage();
    wristAppliedCurrent = wristTalon.getStatorCurrent();
    wristVelocity = wristTalon.getVelocity();
    wristPosition = wristTalon.getPosition();
    topSoftLimit = wristTalon.getFault_ForwardSoftLimit();
    bottomSoftLimit = wristTalon.getFault_ReverseSoftLimit();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        currentPidSlot,
        wristAppliedVoltage,
        wristAppliedCurrent,
        wristVelocity,
        wristPosition,
        topSoftLimit,
        bottomSoftLimit);

    ParentDevice.optimizeBusUtilizationForAll(wristTalon);
  }

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

    // Is connected
    inputs.motorConnected = wristConnectedDebouncer.calculate(wristSignals.isOK());
    inputs.cancoderConnected = wristConnectedDebouncer.calculate(wristEncoder.isConnected());

    // PID slot
    inputs.pidSlot = currentPidSlot.getValue();

    // Voltage
    inputs.motorAppliedVolts = wristAppliedVoltage.getValue();

    // Current
    inputs.motorCurrent = wristAppliedCurrent.getValue();

    // Velocity
    inputs.rotationVelocity = wristVelocity.getValue();

    // Position
    inputs.mechanismAngle = wristPosition.getValue();

    // Limits
    inputs.topLimit = topSoftLimit.getValue();
    inputs.bottomLimit = bottomSoftLimit.getValue();
  }

  @Override
  public void setWristPosition(Angle rotation, int pidSlot) {
    wristTalon.setControl(positionVoltage.withPosition(rotation).withSlot(pidSlot));
  }

  @Override
  public void setWristOpenLoop(double percentOutput) {
    wristTalon.set(percentOutput);
  }

  @Override
  public void stop() {
    wristTalon.stopMotor();
  }
}
