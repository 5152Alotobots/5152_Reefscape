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

import static frc.alotobots.Constants.CanId.SERVO_HUB_CAN_ID;
import static frc.alotobots.reefscape.subsystems.climber.constants.ClimberRevServoReal.LOCKING_SERVO_ID;
import static frc.alotobots.reefscape.subsystems.climber.constants.ClimberRevServoReal.PLUNGER_SERVO_ID;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;

public class ClimberIORevServoReal implements ClimberIO {
  private final ServoHub servoHub = new ServoHub(SERVO_HUB_CAN_ID);
  private final ServoChannel plungerServoChannel = servoHub.getServoChannel(PLUNGER_SERVO_ID);
  private final ServoChannel lockingServoChannel = servoHub.getServoChannel(LOCKING_SERVO_ID);

  ClimberIORevServoReal() {}

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.servoHubConnected = true;
    inputs.lockingServoPosition = lockingServoChannel.getPulseWidth();
    inputs.plungerServoPosition = plungerServoChannel.getPulseWidth();
  }

  @Override
  public void togglePlungerServoEnabled() {
    if (!plungerServoChannel.isEnabled()) {
      plungerServoChannel.setEnabled(true);
      plungerServoChannel.setPowered(true);
    } else {
      plungerServoChannel.setEnabled(false);
      plungerServoChannel.setPowered(false);
    }
  }

  @Override
  public void toggleLockingServoEnabled() {
    if (!lockingServoChannel.isEnabled()) {
      lockingServoChannel.setEnabled(true);
      lockingServoChannel.setPowered(true);
    } else {
      lockingServoChannel.setEnabled(false);
      lockingServoChannel.setPowered(false);
    }
  }

  @Override
  public void setPlungerServoPosition() {
    plungerServoChannel.setPulseWidth(1400);
  }

  @Override
  public void setLockingServoPosition() {
    lockingServoChannel.setPulseWidth(1600);
  }
}
