/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.bling.io;

import static frc.alotobots.Constants.CanId.CANDLE_CAN_ID;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.*;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

/**
 * Hardware implementation of the BlingIO interface for controlling physical LED strips. Uses CTRE's
 * CANdle device for LED control.
 */
public class BlingIOReal implements BlingIO {
  /** CANdle controller for LED management */
  private CANdle candle;

  /** Current solid color setting */
  private LoggedColor currentColor;

  /** Current animation setting */
  private Animation currentAnimation;

  /**
   * Constructs a new BlingIOReal instance. Initializes the CANdle controller with default settings.
   */
  public BlingIOReal() {
    this.candle = new CANdle(CANDLE_CAN_ID);
    candle.configBrightnessScalar(MAX_LED_BRIGHTNESS);
    candle.configLEDType(LED_TYPE);
    candle.configStatusLedState(DISABLE_STATUS_LED);
  }

  @Override
  public void updateInputs(BlingIOInputs inputs) {
    inputs.currentSolidColor = currentColor;
    if (currentAnimation != null) {
      inputs.animationName = currentAnimation.getClass().getSimpleName();
    } else {
      inputs.animationName = "";
    }
    inputs.hasAnimation = currentAnimation != null;
    inputs.hasColor = currentColor != null;
  }

  @Override
  public void setAnimation(Animation animation) {
    currentAnimation = animation;
    candle.animate(animation);
  }

  @Override
  public void clearAnimation() {
    currentAnimation = null;
    candle.clearAnimation(0);
  }

  @Override
  public void setSolidColor(LoggedColor color) {
    currentColor = color;
    candle.setLEDs(color.red(), color.green(), color.blue());
  }

  @Override
  public void clearSolidColor() {
    currentColor = null;
    candle.setLEDs(0, 0, 0);
  }
}
