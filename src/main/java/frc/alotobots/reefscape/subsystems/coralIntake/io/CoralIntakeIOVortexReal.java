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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.Logger;

public class CoralIntakeIOVortexReal implements CoralIntakeIO {

  private final SparkFlex intakeMotor = new SparkFlex(INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
  private final CANrange intakeSensor = new CANrange(INTAKE_CANRANGE_ID);

  private final StatusSignal<Boolean> intakeOccupied = intakeSensor.getIsDetected();

  public CoralIntakeIOVortexReal() {
    SparkFlexConfig config = new SparkFlexConfig();
    // SparkPIDController =

    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    // PID Configuration (Optional, for velocity control or precise movement)
    // intakePID.
    // intakePID.setI(0.0);
    // intakePID.setD(0.0);
    // intakePID.setFF(0.0);  //  Important:  You *must* tune this if using velocity control.
    // intakePID.setOutputRange(-1.0, 1.0); //  Good practice to limit output.
    intakeMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, intakeOccupied);
    ParentDevice.optimizeBusUtilizationForAll(intakeSensor);
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(intakeOccupied);
    Logger.recordOutput("Temp/Occ", intakeOccupied.getValue());
    inputs.intakeOccupied = intakeOccupied.getValue();
    inputs.velocity = RevolutionsPerSecond.of((intakeMotor.getEncoder().getVelocity()) / 60);
    inputs.motorAppliedVolts =
        Volts.of(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    inputs.motorCurrentAmps = Amps.of(intakeMotor.getOutputCurrent());
  }

  @Override
  public void setIntakeOpenLoop(double percent) {
    intakeMotor.set(percent);
  }

  @Override
  public boolean getIntakeOccupied() {
    return intakeOccupied.getValue();
  }

  // @Override
  // public void setIntakeVelocity(double velocityRPM) {
  // Use the velocity
  //     intakePID.setReference(velocityRPM, ControlType.kVelocity, 0, 0, ArbFFUnits.kVoltage); //
  // kVelocity, PID Slot, Arb Feedforward, Arb FF Units
  // }

  @Override
  public void stop() {
    intakeMotor.set(0.0);
  }
}
