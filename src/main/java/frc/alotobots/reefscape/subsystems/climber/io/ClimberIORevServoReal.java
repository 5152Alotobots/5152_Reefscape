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

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.Constants.CanId.SERVO_HUB_CAN_ID;
import static frc.alotobots.reefscape.subsystems.climber.constants.ClimberRevServoRealConstants.*;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIORevServoReal implements ClimberIO {
  private final ServoHub servoHub = new ServoHub(SERVO_HUB_CAN_ID);
  private final ServoChannel plungerServoChannel = servoHub.getServoChannel(PLUNGER_SERVO_ID);
  private final ServoChannel lockingServoChannel = servoHub.getServoChannel(LOCKING_SERVO_ID);
  private final DigitalInput cageSwitch1 = new DigitalInput(CAGE_SWITCH_1_ID);
  private final DigitalInput cageSwitch2 = new DigitalInput(CAGE_SWITCH_2_ID);

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
    inputs.servoHubVoltage = Volts.of(servoHub.getDeviceVoltage());
    inputs.servoHubCurrent = Amps.of(servoHub.getDeviceCurrent());

    inputs.servoHubServoVoltage = Volts.of(servoHub.getServoVoltage());

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

  /**
   * Sets the plunger servo position based on an angle.
   *
   * @param angle The desired angle, where: - 0 degrees = down/plunge position (PLUNGER_SERVO_0_PW)
   *     - 180 degrees = up/receive position (PLUNGER_SERVO_180_PW) The angle is mapped to pulse
   *     width: - Maps 0° → PLUNGER_SERVO_0_PW (plunge/down position) - Maps 180° →
   *     PLUNGER_SERVO_180_PW (receive/up position)
   */
  @Override
  public void setPlungerServoPosition(Angle angle) {
    plungerServoChannel.setPulseWidth(
        (int)
            ((angle.in(Rotations) * (PLUNGER_SERVO_180_PW - PLUNGER_SERVO_0_PW))
                + PLUNGER_SERVO_0_PW));
  }

  @Override
  public void setLockingServoLocked(boolean lockingServoLocked) {
    if (lockingServoLocked) lockingServoChannel.setPulseWidth(LOCKING_SERVO_CLOSED_PW);
    else lockingServoChannel.setPulseWidth(LOCKING_SERVO_OPEN_PW);
  }
}
