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
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.alotobots.Constants.CanId.WRIST_ENCODER_CAN_ID;
import static frc.alotobots.Constants.CanId.WRIST_MOTOR_CAN_ID;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.MAX_ANGLE;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants.Limits.MIN_ANGLE;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXRealConstants.ROTOR_TO_SENSOR_RATIO;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXSimConstants.ARM_LENGTH;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXSimConstants.INERTIA_KGM2;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXSimConstants.MOTOR_DIRECTION;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXRealConstants;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristTalonFXSimConstants.MotionMagicConstants;

/**
 * Hardware implementation of the WristIO interface using TalonFX motor controller and CANCoder for
 * real robot operation. This class manages the physical wrist mechanism, handling motor control,
 * position sensing, and safety limits.
 */
public class WristIOTalonFXSim implements WristIO {

  private final TalonFX wristTalon = new TalonFX(WRIST_MOTOR_CAN_ID);
  private final CANcoder wristEncoder = new CANcoder(WRIST_ENCODER_CAN_ID);
  private final CANcoderSimState encoderSim = wristEncoder.getSimState();

  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final MotionMagicVoltage magicPositionVoltage = new MotionMagicVoltage(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final DCMotor motor = DCMotor.getFalcon500(1);
  private final TalonFXSimState motorSim = wristTalon.getSimState();

  private final SingleJointedArmSim wristSim =
      new SingleJointedArmSim(
          motor,
          ROTOR_TO_SENSOR_RATIO,
          INERTIA_KGM2,
          ARM_LENGTH,
          MIN_ANGLE.in(Radians),
          MAX_ANGLE.in(Radians),
          false,
          0);

  /**
   * Creates a new WristIOTalonFXReal instance and configures all motor controller and encoder
   * settings. This includes PID configurations, software limits, current limits, and sensor
   * settings.
   */
  public WristIOTalonFXSim() {
    var wristMotorConfig = new TalonFXConfiguration();

    configPIDgains(wristMotorConfig);

    wristMotorConfig.MotorOutput.NeutralMode = WristTalonFXRealConstants.MECHANISM_NEUTRAL_MODE;

    wristMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    wristMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    wristMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);
    wristMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);
    wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristMotorConfig.Feedback.FeedbackRemoteSensorID = WRIST_ENCODER_CAN_ID;
    wristMotorConfig.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;

    wristMotorConfig.MotorOutput.Inverted = MOTOR_DIRECTION;

    wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        MotionMagicConstants.CRUISE_VELOCITY.in(RotationsPerSecond);
    wristMotorConfig.MotionMagic.MotionMagicAcceleration =
        MotionMagicConstants.ACCELERATION.in(RotationsPerSecondPerSecond);
    wristMotorConfig.MotionMagic.MotionMagicJerk = MotionMagicConstants.JERK;

    // Configure CANcoder for simulation
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    motorSim.Orientation = ChassisReference.Clockwise_Positive;
    encoderSim.Orientation = ChassisReference.CounterClockwise_Positive;

    tryUntilOk(5, () -> wristTalon.getConfigurator().apply(wristMotorConfig, 0.25));
    tryUntilOk(5, () -> wristEncoder.getConfigurator().apply(encoderConfig, 0.25));
  }

  private void configPIDgains(TalonFXConfiguration wristMotorConfig) {
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
  }

  /**
   * Updates the input values for the wrist subsystem by reading the latest status from hardware.
   * This includes position, velocity, current, voltage, and limit switch states.
   *
   * @param inputs The WristIOInputs object to update with the latest hardware state
   */
  @Override
  public void updateInputs(WristIOInputs inputs) {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    wristSim.setInputVoltage(motorSim.getMotorVoltage());
    wristSim.update(0.02); // Assuming 20ms update rate

    // Sync motor simulation with arm simulation
    motorSim.setRawRotorPosition((wristSim.getAngleRads() / (2 * Math.PI)) * ROTOR_TO_SENSOR_RATIO);
    motorSim.setRotorVelocity(
        (wristSim.getVelocityRadPerSec() / (2 * Math.PI)) * ROTOR_TO_SENSOR_RATIO);

    // Update the CANcoder simulation with the mechanism position
    encoderSim.setRawPosition(wristSim.getAngleRads() / (2 * Math.PI));
    encoderSim.setVelocity(wristSim.getVelocityRadPerSec() / (2 * Math.PI));

    // Update all input values
    inputs.motorAppliedVolts = Volts.of(motorSim.getMotorVoltage());
    inputs.motorCurrent = Amps.of(motorSim.getTorqueCurrent());
    inputs.rotationVelocity = RadiansPerSecond.of(wristSim.getVelocityRadPerSec());
    inputs.mechanismAngle = Radians.of(wristSim.getAngleRads());
    inputs.topLimit = wristSim.getAngleRads() >= MAX_ANGLE.in(Radians);
    inputs.bottomLimit = wristSim.getAngleRads() <= MIN_ANGLE.in(Radians);
  }

  /**
   * Sets the wrist to a target position using closed-loop control.
   *
   * @param position The target angle to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  @Override
  public void setWristPosition(Angle position, int pidSlot) {

    // Set up the request with appropriate limits
    wristTalon.setControl(positionVoltage.withPosition(position).withSlot(pidSlot));
  }

  /**
   * Sets the wrist to a target position using closed-loop control and motion magic.
   *
   * @param position The target angle to move to
   * @param pidSlot The PID slot to use (0 for velocity, 1 for position)
   */
  @Override
  public void setWristPositionMotionMagic(Angle position, int pidSlot) {
    // Get current angle to determine if we need to apply limits

    // Set up the request with appropriate limits
    wristTalon.setControl(magicPositionVoltage.withPosition(position).withSlot(pidSlot));
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

    // Determine if we should activate limits

    wristTalon.setControl(dutyCycleOut.withOutput(percentOutput));
  }

  /** Stops the wrist motor by setting the motor output to zero. */
  @Override
  public void stop() {
    wristTalon.stopMotor();
  }
}
