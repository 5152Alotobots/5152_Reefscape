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

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class CoralIntakeIOVortexReal implements CoralIntakeIO {

  private final SparkFlex intakeMotor = new SparkFlex(0, MotorType.kBrushless);
  private final CANrange intakeSensor = new CANrange(0);

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
    // intakeMotor.burnFlash();

  }

  // @Override
  // public void updateInputs(IntakeIOInputs inputs) {
  //     inputs.intakeSpeed = intakeEncoder.getVelocity();
  //     inputs.intakeAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  //     inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
  //     inputs.intakeTempCelsius = intakeMotor.getMotorTemperature();
  //     inputs.intakeSensorTripped = !intakeSensor.get(); //  Inverted because DigitalInput.get()
  // is false when tripped.
  // }

  // @Override
  // public void setIntakeVoltage(double volts) {
  //     intakeMotor.setVoltage(volts);
  // }

  // @Override
  // public void setIntakePercent(double percent) {
  //     intakeMotor.set(percent);
  // }

  // @Override
  // public void setIntakeVelocity(double velocityRPM) {
  //     //Use the velocity
  //     intakePID.setReference(velocityRPM, ControlType.kVelocity, 0, 0, ArbFFUnits.kVoltage); //
  // kVelocity, PID Slot, Arb Feedforward, Arb FF Units
  // }

  // @Override
  // public void stopIntake() {
  //     intakeMotor.set(0.0);
  // }

  // @Override
  // public boolean isIntakeSensorTripped(){
  //     return !intakeSensor.get();
  // }

  // @Override
  // public void periodic() {
  //     // This method will be called once per scheduler run
  //     // Log any values to SmartDashboard or Shuffleboard here.
  //     SmartDashboard.putNumber("Intake Motor Speed", intakeEncoder.getVelocity());
  //     SmartDashboard.putNumber("Intake Motor Current", intakeMotor.getOutputCurrent());
  //     SmartDashboard.putBoolean("Intake Sensor Tripped", !intakeSensor.get()); //  Inverted for
  // display.
  // }
}
