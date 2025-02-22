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
package frc.alotobots.reefscape.subsystems.climber.io;

import static edu.wpi.first.units.Units.Degrees;
import static frc.alotobots.Constants.CanId.SERVO_HUB_CAN_ID;
import static frc.alotobots.reefscape.subsystems.climber.constants.ClimberRevServoReal.*;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIORevServoReal implements ClimberIO {
  private final ServoHub servoHub = new ServoHub(SERVO_HUB_CAN_ID);
  private final ServoChannel plungerServoChannel = servoHub.getServoChannel(PLUNGER_SERVO_ID);
  private final ServoChannel lockingServoChannel = servoHub.getServoChannel(LOCKING_SERVO_ID);
  private final DigitalInput cageSwitch1 = new DigitalInput(0);
  private final DigitalInput cageSwitch2 = new DigitalInput(1);

  public ClimberIORevServoReal() {
    enablePlungerServo();
    enableLockingServo();

    disablePlungerServo();
    disableLockingServo();
  }

  @Override
  public boolean getCageSwitches() {
    return cageSwitch1.get() && cageSwitch2.get();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.plungerServoEnabled = plungerServoChannel.isEnabled();
    inputs.lockingServoEnabled = lockingServoChannel.isEnabled();

    inputs.cageSwitch1 = cageSwitch1.get();
    inputs.cageSwitch2 = cageSwitch2.get();

    inputs.servoHubConnected = true;
    inputs.lockingServoPulseWidth = lockingServoChannel.getPulseWidth();
    inputs.plungerServoPulseWidth = plungerServoChannel.getPulseWidth();
  }

  @Override
  public void enablePlungerServo() {
    plungerServoChannel.setEnabled(true);
    plungerServoChannel.setPowered(true);
  }

  @Override
  public void enableLockingServo() {
    lockingServoChannel.setEnabled(true);
    lockingServoChannel.setPowered(true);
  }

  @Override
  public void disablePlungerServo() {
    plungerServoChannel.setEnabled(false);
    plungerServoChannel.setPowered(false);
  }

  @Override
  public void disableLockingServo() {
    lockingServoChannel.setEnabled(false);
    lockingServoChannel.setPowered(false);
  }

  @Override
  public void setPlungerServoPosition(Angle angle) {
    plungerServoChannel.setPulseWidth((int) (((angle.in(Degrees) / 360) * 1850) + 650));
  }

  @Override
  public void setLockingServoLocked(boolean lockingServoLocked) {
    if (lockingServoLocked) lockingServoChannel.setPulseWidth(LOCKING_SERVO_CLOSED_PW);
    else lockingServoChannel.setPulseWidth(LOCKING_SERVO_OPEN_PW);
  }
}
