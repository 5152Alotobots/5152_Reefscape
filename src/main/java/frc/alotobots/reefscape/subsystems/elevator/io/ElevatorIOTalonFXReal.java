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
import static frc.alotobots.Constants.CanId.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants.Limits.*;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorTalonFXRealConstants.LEFT_MOTOR_DIRECTION;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorTalonFXRealConstants.MECHANISM_NEUTRAL_MODE;
import static frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorTalonFXRealConstants.MotorSafetyLimits.*;
import static frc.alotobots.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorTalonFXRealConstants;

/**
 * Hardware implementation of the Elevator subsystem using TalonFX motors and a CANRange sensor.
 * This class manages two TalonFX motors (left and right) and a CANRange for position feedback. The
 * right motor follows the left motor in an inverted configuration to ensure synchronized movement.
 * The CANRange provides absolute position feedback for the elevator mechanism.
 */
public class ElevatorIOTalonFXReal implements ElevatorIO {

  /** The primary TalonFX motor controller for the elevator */
  private final TalonFX leftTalon;

  /** The follower TalonFX motor controller for the elevator */
  private final TalonFX rightTalon;

  /** The CANRange sensor for absolute position feedback */
  private final CANrange canRange;

  /** Position voltage control request for standard position-based control */
  private final PositionVoltage positionVoltage = new PositionVoltage(0.0);

  /** Position torque current FOC control request for position control with torque management */
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0);

  /** Status signal for the current PID slot */
  private final StatusSignal<Integer> currentPidSlot;

  /** Status signal for the left motor's applied voltage */
  private final StatusSignal<Voltage> leftAppliedVoltage;

  /** Status signal for the left motor's applied current */
  private final StatusSignal<Current> leftAppliedCurrent;

  /** Status signal for the left motor's velocity */
  private final StatusSignal<AngularVelocity> leftVelocity;

  /** Status signal for the left motor's position */
  private final StatusSignal<Angle> leftPosition;

  /** Status signal for the right motor's applied voltage */
  private final StatusSignal<Voltage> rightAppliedVoltage;

  /** Status signal for the right motor's applied current */
  private final StatusSignal<Current> rightAppliedCurrent;

  /** Status signal for the right motor's velocity */
  private final StatusSignal<AngularVelocity> rightVelocity;

  /** Status signal for the right motor's position */
  private final StatusSignal<Angle> rightPosition;

  /** Status signal for the CANRange's absolute position measurement */
  private final StatusSignal<Distance> canRangeDistance;

  /** Status signal indicating if the top soft limit is reached */
  private final StatusSignal<Boolean> topSoftLimit;

  /** Status signal indicating if the bottom soft limit is reached */
  private final StatusSignal<Boolean> bottomSoftLimit;

  /** Status signal reporting the closed loop error from the master (LEFT) motor */
  private final StatusSignal<Double> closedLoopError;

  /** Debouncer for filtering left motor connection status */
  private final Debouncer leftConnectedDebounce = new Debouncer(0.5);

  /** Debouncer for filtering right motor connection status */
  private final Debouncer rightConnectedDebounce = new Debouncer(0.5);

  /** Debouncer for filtering CANcoder connection status */
  private final Debouncer cancoderConnectedDebounce = new Debouncer(0.5);

  /**
   * Constructs a new ElevatorIOTalonFXReal instance. Initializes and configures the TalonFX motors
   * and CANRange sensor with appropriate settings for position control, current limits, and safety
   * features. The right motor is configured to follow the left motor in an inverted configuration.
   */
  public ElevatorIOTalonFXReal() {
    leftTalon = new TalonFX(LEFT_ELEVATOR_CAN_ID);
    rightTalon = new TalonFX(RIGHT_ELEVATOR_CAN_ID);
    canRange = new CANrange(ELEVATOR_CANRANGE_ID);

    // Left motor config
    var leftConfig = new TalonFXConfiguration();

    // PID configuration for empty mode (Slot 0)
    leftConfig.Slot0.kP = ElevatorTalonFXRealConstants.PIDConstants.EmptyPIDConstants.KP;
    leftConfig.Slot0.kI = ElevatorTalonFXRealConstants.PIDConstants.EmptyPIDConstants.KI;
    leftConfig.Slot0.kD = ElevatorTalonFXRealConstants.PIDConstants.EmptyPIDConstants.KD;
    leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    leftConfig.Slot0.kA = ElevatorTalonFXRealConstants.PIDConstants.EmptyPIDConstants.KA;
    leftConfig.Slot0.kG = ElevatorTalonFXRealConstants.PIDConstants.EmptyPIDConstants.KG;
    leftConfig.Slot0.kS = ElevatorTalonFXRealConstants.PIDConstants.EmptyPIDConstants.KS;
    leftConfig.Slot0.kV = ElevatorTalonFXRealConstants.PIDConstants.EmptyPIDConstants.KV;

    // PID configuration for Coral mode (Slot 1)
    leftConfig.Slot1.kP = ElevatorTalonFXRealConstants.PIDConstants.CoralPIDConstants.KP;
    leftConfig.Slot1.kI = ElevatorTalonFXRealConstants.PIDConstants.CoralPIDConstants.KI;
    leftConfig.Slot1.kD = ElevatorTalonFXRealConstants.PIDConstants.CoralPIDConstants.KD;
    leftConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    leftConfig.Slot1.kA = ElevatorTalonFXRealConstants.PIDConstants.CoralPIDConstants.KA;
    leftConfig.Slot1.kG = ElevatorTalonFXRealConstants.PIDConstants.CoralPIDConstants.KG;
    leftConfig.Slot1.kS = ElevatorTalonFXRealConstants.PIDConstants.CoralPIDConstants.KS;
    leftConfig.Slot1.kV = ElevatorTalonFXRealConstants.PIDConstants.CoralPIDConstants.KV;

    // PID configuration for Algae mode (Slot 2)
    leftConfig.Slot2.kP = ElevatorTalonFXRealConstants.PIDConstants.AlgaePIDConstants.KP;
    leftConfig.Slot2.kI = ElevatorTalonFXRealConstants.PIDConstants.AlgaePIDConstants.KI;
    leftConfig.Slot2.kD = ElevatorTalonFXRealConstants.PIDConstants.AlgaePIDConstants.KD;
    leftConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    leftConfig.Slot2.kA = ElevatorTalonFXRealConstants.PIDConstants.AlgaePIDConstants.KA;
    leftConfig.Slot2.kG = ElevatorTalonFXRealConstants.PIDConstants.AlgaePIDConstants.KG;
    leftConfig.Slot2.kS = ElevatorTalonFXRealConstants.PIDConstants.AlgaePIDConstants.KS;
    leftConfig.Slot2.kV = ElevatorTalonFXRealConstants.PIDConstants.AlgaePIDConstants.KV;

    leftConfig.MotorOutput.NeutralMode = MECHANISM_NEUTRAL_MODE;

    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = LIMITS_ENABLED;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = LIMITS_ENABLED;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        heightToTalonFX(MAX_HEIGHT).in(Rotations);
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        heightToTalonFX(MIN_HEIGHT).in(Rotations);

    leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    leftConfig.TorqueCurrent.PeakForwardTorqueCurrent = TORQUE_FORWARD_AMP_LIMIT.in(Amps);
    leftConfig.TorqueCurrent.PeakReverseTorqueCurrent = TORQUE_REVERSE_AMP_LIMIT.in(Amps);

    leftConfig.CurrentLimits.StatorCurrentLimit = STATOR_AMP_LIMIT.in(Amps);
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Always should be true

    leftConfig.MotorOutput.Inverted = LEFT_MOTOR_DIRECTION;

    // Apply config to left motor
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftConfig, 0.25));

    // Set right motor to be inverted follower
    tryUntilOk(5, () -> rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true)));

    // CANRange config
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
    canRangeConfig.ToFParams.UpdateFrequency = 50; // Hz
    canRangeConfig.FovParams.FOVRangeX = 6.75;
    canRangeConfig.FovParams.FOVRangeY = 6.75;

    // Apply config to CANRange
    tryUntilOk(5, () -> canRange.getConfigurator().apply(canRangeConfig, 0.25));

    currentPidSlot = leftTalon.getClosedLoopSlot();

    leftPosition = leftTalon.getPosition();
    rightPosition = rightTalon.getPosition();
    canRangeDistance = canRange.getDistance();

    leftVelocity = leftTalon.getVelocity();
    rightVelocity = rightTalon.getVelocity();

    leftAppliedVoltage = leftTalon.getMotorVoltage();
    rightAppliedVoltage = rightTalon.getMotorVoltage();

    leftAppliedCurrent = leftTalon.getStatorCurrent();
    rightAppliedCurrent = rightTalon.getStatorCurrent();

    topSoftLimit = leftTalon.getFault_ForwardSoftLimit();
    bottomSoftLimit = rightTalon.getFault_ReverseSoftLimit();

    closedLoopError = leftTalon.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        rightPosition,
        canRangeDistance,
        leftVelocity,
        rightVelocity,
        leftAppliedVoltage,
        rightAppliedVoltage,
        leftAppliedCurrent,
        rightAppliedCurrent,
        topSoftLimit,
        bottomSoftLimit,
        currentPidSlot,
        closedLoopError);
    ParentDevice.optimizeBusUtilizationForAll(leftTalon, rightTalon, canRange);
  }

  /**
   * Updates the input values for the elevator subsystem. Refreshes all status signals and updates
   * the provided inputs object with current sensor readings and state information.
   *
   * @param inputs The ElevatorIOInputs object to update with current values
   */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var leftSignals =
        BaseStatusSignal.refreshAll(
            leftPosition,
            leftVelocity,
            leftAppliedVoltage,
            leftAppliedCurrent,
            topSoftLimit,
            bottomSoftLimit,
            closedLoopError);
    var rightSignals =
        BaseStatusSignal.refreshAll(
            rightPosition, rightVelocity, rightAppliedVoltage, rightAppliedCurrent);
    var canRangeSignals = BaseStatusSignal.refreshAll(canRangeDistance);

    // Current slot
    inputs.currentPidSlot = currentPidSlot.getValue();

    // Connected status
    inputs.leftMotorConnected = leftConnectedDebounce.calculate(leftSignals.isOK());
    inputs.rightMotorConnected = rightConnectedDebounce.calculate(rightSignals.isOK());
    inputs.canrangeConnected = cancoderConnectedDebounce.calculate(canRangeSignals.isOK());

    // Limits
    inputs.topLimit = topSoftLimit.getValue();
    inputs.bottomLimit = bottomSoftLimit.getValue();

    // Positions
    inputs.leftHeight = talonFXToHeight(leftPosition.getValue());
    inputs.rightHeight = talonFXToHeight(rightPosition.getValue());
    inputs.leftMotorAngle = leftPosition.getValue();
    inputs.rightMotorAngle = rightPosition.getValue();
    inputs.canrangeDistance = canRangeDistance.getValue();

    // Velocities
    inputs.leftVelocity = talonFXToLinearVelocity(leftVelocity.getValue());
    inputs.rightVelocity = talonFXToLinearVelocity(rightVelocity.getValue());

    // Volts
    inputs.leftAppliedVolts = leftAppliedVoltage.getValue();
    inputs.rightAppliedVolts = rightAppliedVoltage.getValue();

    // Amps
    inputs.leftCurrentAmps = leftAppliedCurrent.getValue();
    inputs.rightCurrentAmps = rightAppliedCurrent.getValue();

    // Closed Loop General
    inputs.mechanismClosedLoopError = talonFXToHeight(Rotations.of(closedLoopError.getValue()));
  }

  /**
   * Sets the elevator to a specific position using closed-loop control.
   *
   * @param position The target position as a Distance unit
   * @param pidSlot The PID slot to use (0 for Coral mode, 1 for Algae mode)
   */
  @Override
  public void setElevatorPosition(Distance position, int pidSlot) {
    leftTalon.setControl(positionVoltage.withPosition(heightToTalonFX(position)).withSlot(pidSlot));
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

  /** Stops all elevator motor movement. */
  @Override
  public void stop() {
    leftTalon.stopMotor();
  }

  /**
   * Converts TalonFX rotations to elevator height. TalonFX reports position in rotations in Phoenix
   * 6. Uses regression formula y = 0.00942151x + 0.26 where x is rotations and y is meters.
   *
   * @param rotations TalonFX motor rotations
   * @return Height as a Distance unit
   */
  private Distance talonFXToHeight(Angle rotations) {
    return Meters.of(0.00942151 * rotations.in(Rotations) + MIN_HEIGHT.in(Meters));
  }

  /**
   * Converts TalonFX rotational velocity to linear velocity. TalonFX reports velocity in rotations
   * per second in Phoenix 6. Uses slope from regression formula y = 0.00942151x + 0.26.
   *
   * @param rotationsPerSecond TalonFX motor rotational velocity
   * @return Linear velocity as a LinearVelocity unit
   */
  private LinearVelocity talonFXToLinearVelocity(AngularVelocity rotationsPerSecond) {
    return MetersPerSecond.of(rotationsPerSecond.in(RotationsPerSecond) * 0.00942151);
  }

  /**
   * Converts elevator height to TalonFX rotations. TalonFX expects position in rotations in Phoenix
   * 6. Uses inverse of regression formula y = 0.00942151x + 0.26, solving for x: x = (y - 0.26) /
   * 0.00949501 where y is meters and x is rotations.
   *
   * @param height Height as a Distance unit
   * @return TalonFX motor rotations as an Angle unit
   */
  private Angle heightToTalonFX(Distance height) {
    return Rotations.of((height.minus(MIN_HEIGHT).in(Meters)) / 0.00942151);
  }

  /**
   * Converts linear velocity to TalonFX rotational velocity. TalonFX expects velocity in rotations
   * per second in Phoenix 6. Uses inverse slope from regression formula y = 0.00942151x + 0.26.
   *
   * @param linearVelocity Linear velocity as a LinearVelocity unit
   * @return TalonFX motor rotational velocity as an AngularVelocity unit
   */
  private AngularVelocity linearVelocityToTalonFX(LinearVelocity linearVelocity) {
    return RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / 0.00942151);
  }
}
