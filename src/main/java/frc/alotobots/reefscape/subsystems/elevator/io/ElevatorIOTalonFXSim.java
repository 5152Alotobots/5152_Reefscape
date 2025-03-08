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
package frc.alotobots.reefscape.subsystems.elevator.io;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.Constants.CanId.LEFT_ELEVATOR_CAN_ID;
import static frc.alotobots.Constants.CanId.RIGHT_ELEVATOR_CAN_ID;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.LIMITS_ENABLED;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorTalonFXRealConstants.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorTalonFXRealConstants.MotorSafetyLimits.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorTalonFXSimConstants.HardwareConstants.*;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

/** Motor Control Types */
public class ElevatorIOTalonFXSim implements ElevatorIO {
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
  private final MotionMagicVoltage magicPositionVoltage = new MotionMagicVoltage(0.0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

  private final ElevatorSim elevatorSim;
  private final DCMotor motors = DCMotor.getFalcon500(2);
  private final TalonFX leftTalon = new TalonFX(LEFT_ELEVATOR_CAN_ID);
  private final TalonFX rightTalon = new TalonFX(RIGHT_ELEVATOR_CAN_ID);

  private final StatusSignal<Integer> currentPidSlot;
  private final StatusSignal<Voltage> leftAppliedVoltage;
  private final StatusSignal<Current> leftAppliedCurrent;
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<AngularAcceleration> leftAcceleration;
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<Voltage> rightAppliedVoltage;
  private final StatusSignal<Current> rightAppliedCurrent;
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<AngularAcceleration> rightAcceleration;
  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<Boolean> topSoftLimit;
  private final StatusSignal<Boolean> bottomSoftLimit;

  private final TalonFXSimState leftSim = leftTalon.getSimState();
  private final TalonFXSimState rightSim = rightTalon.getSimState();

  public ElevatorIOTalonFXSim() {
    // Left motor config
    var leftConfig = new TalonFXConfiguration();

    configPIDGains(leftConfig);

    leftConfig.MotorOutput.NeutralMode = MECHANISM_NEUTRAL_MODE;

    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = LIMITS_ENABLED;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = LIMITS_ENABLED;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        heightToTalonFX(MAX_HEIGHT).in(Rotations);
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        heightToTalonFX(MIN_HEIGHT).in(Rotations);

    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    leftConfig.MotionMagic.MotionMagicCruiseVelocity =
        linearVelocityToTalonFX(MotionMagicConstants.CRUISE_VELOCITY).in(RotationsPerSecond);
    leftConfig.MotionMagic.MotionMagicAcceleration =
        linearAccelerationToTalonFX(MotionMagicConstants.ACCELERATION)
            .in(RotationsPerSecondPerSecond);
    leftConfig.MotionMagic.MotionMagicJerk = MotionMagicConstants.JERK;

    // Apply config to left motor
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftConfig, 0.25));

    // Set right motor to be inverted follower
    tryUntilOk(5, () -> rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true)));

    leftSim.Orientation = ChassisReference.Clockwise_Positive;
    rightSim.Orientation = ChassisReference.CounterClockwise_Positive;

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

    topSoftLimit = leftTalon.getFault_ForwardSoftLimit();
    bottomSoftLimit = leftTalon.getFault_ReverseSoftLimit();

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
        topSoftLimit,
        bottomSoftLimit,
        currentPidSlot);

    elevatorSim =
        new ElevatorSim(
            motors,
            GEAR_RATIO,
            ELEVATOR_MASS_KG,
            PULLEY_RADIUS_M, // Convert circumference to radius
            MIN_HEIGHT.in(Meters),
            MAX_HEIGHT.in(Meters),
            true,
            0.0);
  }

  private static void configPIDGains(TalonFXConfiguration leftConfig) {
    // PID configuration for velocity mode (Slot 0)
    leftConfig.Slot0.kP = PIDConstants.VelocityPIDConstants.KP;
    leftConfig.Slot0.kI = PIDConstants.VelocityPIDConstants.KI;
    leftConfig.Slot0.kD = PIDConstants.VelocityPIDConstants.KD;
    leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    leftConfig.Slot0.kA = PIDConstants.VelocityPIDConstants.KA;
    leftConfig.Slot0.kG = PIDConstants.VelocityPIDConstants.KG;
    leftConfig.Slot0.kS = PIDConstants.VelocityPIDConstants.KS;
    leftConfig.Slot0.kV = PIDConstants.VelocityPIDConstants.KV;

    // PID configuration for position mode (Slot 1)
    leftConfig.Slot1.kP = PIDConstants.PositionPIDConstants.KP;
    leftConfig.Slot1.kI = PIDConstants.PositionPIDConstants.KI;
    leftConfig.Slot1.kD = PIDConstants.PositionPIDConstants.KD;
    leftConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    leftConfig.Slot1.kA = PIDConstants.PositionPIDConstants.KA;
    leftConfig.Slot1.kG = PIDConstants.PositionPIDConstants.KG;
    leftConfig.Slot1.kS = PIDConstants.PositionPIDConstants.KS;
    leftConfig.Slot1.kV = PIDConstants.PositionPIDConstants.KV;

    // PID configuration for climbing mode (Slot 2)
    leftConfig.Slot2.kP = PIDConstants.ClimbingPIDConstants.KP;
    leftConfig.Slot2.kI = PIDConstants.ClimbingPIDConstants.KI;
    leftConfig.Slot2.kD = PIDConstants.ClimbingPIDConstants.KD;
    leftConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    leftConfig.Slot2.kA = PIDConstants.ClimbingPIDConstants.KA;
    leftConfig.Slot2.kG = PIDConstants.ClimbingPIDConstants.KG;
    leftConfig.Slot2.kS = PIDConstants.ClimbingPIDConstants.KS;
    leftConfig.Slot2.kV = PIDConstants.ClimbingPIDConstants.KV;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    elevatorSim.update(0.02); // 50Hz simulation rate

    double[] motorVoltage = {leftSim.getMotorVoltage() /*, rightSim.getMotorVoltage()*/};
    elevatorSim.setInput(motorVoltage);

    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAcceleration,
        leftAppliedVoltage,
        leftAppliedCurrent,
        topSoftLimit,
        bottomSoftLimit);
    BaseStatusSignal.refreshAll(
        rightPosition, rightVelocity, rightAcceleration, rightAppliedVoltage, rightAppliedCurrent);

    inputs.currentPidSlot = currentPidSlot.getValue();

    // Connected status
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;

    // Limits
    inputs.topLimit = topSoftLimit.getValue();
    inputs.bottomLimit = bottomSoftLimit.getValue();

    // Positions
    inputs.leftHeight = talonFXToHeight(leftPosition.getValue());
    inputs.rightHeight = talonFXToHeight(rightPosition.getValue());
    inputs.leftMotorAngle = leftPosition.getValue();
    inputs.rightMotorAngle = rightPosition.getValue();

    // Velocities
    inputs.leftVelocity = talonFXToLinearVelocity(leftVelocity.getValue());
    inputs.rightVelocity = talonFXToLinearVelocity(rightVelocity.getValue());

    // Acceleration
    inputs.leftAcceleration = talonFXToLinearAcceleration(leftAcceleration.getValue());
    inputs.rightAcceleration = talonFXToLinearAcceleration(rightAcceleration.getValue());

    // Volts
    inputs.leftAppliedVolts = leftAppliedVoltage.getValue();
    inputs.rightAppliedVolts = rightAppliedVoltage.getValue();

    // Amps
    inputs.leftCurrentAmps = leftAppliedCurrent.getValue();
    inputs.rightCurrentAmps = rightAppliedCurrent.getValue();

    Logger.recordOutput("ElevatorSim/Height", elevatorSim.getPositionMeters());
    Logger.recordOutput("ElevatorSim/Input", elevatorSim.getInput());
    Logger.recordOutput("ElevatorSim/Velocity", elevatorSim.getVelocityMetersPerSecond());
    Logger.recordOutput("ElevatorSim/Current", elevatorSim.getCurrentDrawAmps());
  }

  /**
   * Sets the elevator to a specific position using closed-loop control.
   *
   * @param position The target position as a Distance unit
   * @param pidSlot The PID slot to use (0 for velocity mode, 1 for position mode)
   */
  @Override
  public void setElevatorPosition(Distance position, int pidSlot) {
    leftTalon.setControl(positionVoltage.withPosition(heightToTalonFX(position)).withSlot(pidSlot));
  }

  /**
   * Sets the elevator to a specific position using closed-loop control.
   *
   * @param position The target position as a Distance unit
   * @param pidSlot The PID slot to use (0 for velocity mode, 1 for position mode)
   */
  @Override
  public void setElevatorPositionMotionMagic(Distance position, int pidSlot) {
    leftTalon.setControl(
        magicPositionVoltage.withPosition(heightToTalonFX(position)).withSlot(pidSlot));
  }

  /**
   * Sets the elevator to a specific velocity using closed-loop control.
   *
   * @param velocity The target velocity as a LinearVelocity unit
   * @param pidSlot The PID slot to use (0 for velocity mode, 1 for position mode)
   */
  @Override
  public void setElevatorVelocity(LinearVelocity velocity, int pidSlot) {
    leftTalon.setControl(
        velocityVoltage.withVelocity(linearVelocityToTalonFX(velocity)).withSlot(pidSlot));
  }

  /**
   * Sets the elevator motors to run in open-loop mode at the specified output percentage.
   *
   * @param percentOutput The motor output as a percentage (-1.0 to 1.0)
   */
  @Override
  public void setElevatorOpenLoop(double percentOutput) {
    leftTalon.set(percentOutput);
  }

  /**
   * Sets the brake mode for the elevator motors.
   *
   * @param brake true to enable brake mode, false for coast mode
   */
  @Override
  public void setElevatorBrakeMode(boolean brake) {
    leftTalon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /**
   * Resets the rotor sensors of both motors to be equal to the given height
   *
   * @param height The height off the floor the elevator is at
   */
  @Override
  public void resetRotorPositions(Distance height) {
    leftTalon.setPosition(heightToTalonFX(height));
    rightTalon.setPosition(heightToTalonFX(height));
  }

  /** Stops all elevator motor movement. */
  @Override
  public void stop() {
    leftTalon.stopMotor();
  }

  /**
   * Converts TalonFX rotations to elevator height. TalonFX reports position in rotations in Phoenix
   * 6. Uses regression formula y = HEIGHT_PER_ROTATION + MIN_HEIGHT where x is rotations and y is
   * meters.
   *
   * @param rotations TalonFX motor rotations
   * @return Height as a Distance unit
   */
  private Distance talonFXToHeight(Angle rotations) {
    return Meters.of(HEIGHT_PER_ROTATION * rotations.in(Rotations) + MIN_HEIGHT.in(Meters));
  }

  /**
   * Converts TalonFX rotational velocity to linear velocity. TalonFX reports velocity in rotations
   * per second in Phoenix 6. Uses slope from regression formula y = HEIGHT_PER_ROTATION +
   * MIN_HEIGHT.
   *
   * @param rotationsPerSecond TalonFX motor rotational velocity
   * @return Linear velocity as a LinearVelocity unit
   */
  private LinearVelocity talonFXToLinearVelocity(AngularVelocity rotationsPerSecond) {
    return MetersPerSecond.of(rotationsPerSecond.in(RotationsPerSecond) * HEIGHT_PER_ROTATION);
  }

  /**
   * Converts elevator height to TalonFX rotations. TalonFX expects position in rotations in Phoenix
   * 6. Uses inverse of regression formula y = HEIGHT_PER_ROTATION + MIN_HEIGHT, solving for x: x =
   * (y - MIN_HEIGHT) / HEIGHT_PER_ROTATION where y is meters and x is rotations.
   *
   * @param height Height as a Distance unit
   * @return TalonFX motor rotations as an Angle unit
   */
  private Angle heightToTalonFX(Distance height) {
    return Rotations.of((height.minus(MIN_HEIGHT).in(Meters)) / HEIGHT_PER_ROTATION);
  }

  /**
   * Converts linear velocity to TalonFX rotational velocity. TalonFX expects velocity in rotations
   * per second in Phoenix 6. Uses inverse slope from regression formula y = HEIGHT_PER_ROTATION +
   * MIN_HEIGHT.
   *
   * @param linearVelocity Linear velocity as a LinearVelocity unit
   * @return TalonFX motor rotational velocity as an AngularVelocity unit
   */
  private AngularVelocity linearVelocityToTalonFX(LinearVelocity linearVelocity) {
    return RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / HEIGHT_PER_ROTATION);
  }

  /**
   * Converts linear acceleration to TalonFX rotational acceleration. TalonFX expects acceleration
   * in rotations per second squared in Phoenix 6. Uses the same conversion factor as velocity since
   * acceleration is the time derivative of velocity.
   *
   * @param linearAcceleration Linear acceleration as a LinearAcceleration unit
   * @return TalonFX motor rotational acceleration as an AngularAcceleration unit
   */
  private AngularAcceleration linearAccelerationToTalonFX(LinearAcceleration linearAcceleration) {
    return RotationsPerSecondPerSecond.of(
        linearAcceleration.in(MetersPerSecondPerSecond) / HEIGHT_PER_ROTATION);
  }

  /**
   * Converts TalonFX rotational acceleration to linear acceleration. TalonFX reports acceleration
   * in rotations per second squared in Phoenix 6. Uses the same conversion factor as velocity since
   * acceleration is the time derivative of velocity.
   *
   * @param rotationalAcceleration TalonFX motor rotational acceleration as an AngularAcceleration
   *     unit
   * @return Linear acceleration as a LinearAcceleration unit
   */
  private LinearAcceleration talonFXToLinearAcceleration(
      AngularAcceleration rotationalAcceleration) {
    return MetersPerSecondPerSecond.of(
        rotationalAcceleration.in(RotationsPerSecondPerSecond) * HEIGHT_PER_ROTATION);
  }
}
