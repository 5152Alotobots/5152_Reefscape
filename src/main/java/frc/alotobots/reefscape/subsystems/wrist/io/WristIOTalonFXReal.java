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

import static frc.alotobots.Constants.CanId.WRIST_ENCODER_CAN_ID;
import static frc.alotobots.Constants.CanId.WRIST_MOTOR_CAN_ID;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class WristIOTalonFXReal implements WristIO {

  // Motors
  private final TalonFX wristMotor;
  private final CANcoder wristEncoder;

  // Control Modes
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);

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
    wristMotor = new TalonFX(WRIST_MOTOR_CAN_ID);
    wristEncoder = new CANcoder(WRIST_ENCODER_CAN_ID);

    var wristMotorConfig = new TalonFXConfiguration();

    wristMotorConfig.Feedback.FeedbackRemoteSensorID = WRIST_ENCODER_CAN_ID;
    wristMotorConfig.Feedback.RotorToSensorRatio = 189;

    var wristEncoderConfig = new CANcoderConfiguration();

    wristEncoderConfig.MagnetSensor.MagnetOffset = 0;
    wristEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    // Apply config wrist
    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(wristMotorConfig, 0.25));
    tryUntilOk(5, () -> wristEncoder.getConfigurator().apply(wristEncoderConfig, 0.25));

    currentPidSlot = wristMotor.getClosedLoopSlot();
    wristAppliedVoltage = wristMotor.getMotorVoltage();
    wristAppliedCurrent = wristMotor.getStatorCurrent();
    wristVelocity = wristMotor.getVelocity();
    wristPosition = wristMotor.getPosition();
    topSoftLimit = wristMotor.getFault_ForwardSoftLimit();
    bottomSoftLimit = wristMotor.getFault_ReverseSoftLimit();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        currentPidSlot,
        wristAppliedVoltage,
        wristAppliedCurrent,
        wristVelocity,
        wristPosition,
        topSoftLimit,
        bottomSoftLimit);

    ParentDevice.optimizeBusUtilizationForAll(wristMotor);
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

    // PID slot
    inputs.pidSlot = currentPidSlot.getValue();

    // Voltage
    inputs.motorAppliedVolts = wristAppliedVoltage.getValue();

    // Current
    inputs.motorCurrentAmps = wristAppliedCurrent.getValue();

    // Velocity
    inputs.rotationVelocity = wristVelocity.getValue();

    // Position
    inputs.position = wristPosition.getValue();

    // Limits
    inputs.topLimit = topSoftLimit.getValue();
    inputs.bottomLimit = bottomSoftLimit.getValue();
  }

  @Override
  public void setWristPosition(Angle rotation, int pidSlot) {
    wristMotor.setControl(positionTorqueCurrentFOC.withPosition(rotation).withSlot(pidSlot));
  }

  @Override
  public void setWristOpenLoop(double percentOutput) {
    wristMotor.set(percentOutput);
  }

  @Override
  public void stop() {
    wristMotor.stopMotor();
  }
}
