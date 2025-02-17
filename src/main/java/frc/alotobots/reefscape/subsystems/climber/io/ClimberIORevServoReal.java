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

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;

import static frc.alotobots.Constants.CanId.SERVO_HUB_CAN_ID;

public class ClimberIORevServoReal implements ClimberIO {
  private final ServoHub servoHub = new ServoHub(SERVO_HUB_CAN_ID);


  ClimberIORevServoReal() {
    servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 4500);

  }

  @Override
  public void setPlungerServoPosition() {}

  @Override
  public void setLockingServoPosition() {}
}
