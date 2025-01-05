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

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.*;

import com.ctre.phoenix.led.Animation;

/**
 * Simulation implementation of the BlingIO interface. Provides a mock implementation for testing
 * and simulation purposes.
 */
public class BlingIOSim implements BlingIO {
  /** Current simulated solid color */
  private LoggedColor currentColor;

  /** Current simulated animation */
  private Animation currentAnimation;

  /** Constructs a new BlingIOSim instance. */
  public BlingIOSim() {}

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
  }

  @Override
  public void clearAnimation() {
    currentAnimation = null;
  }

  @Override
  public void setSolidColor(LoggedColor color) {
    currentColor = color;
  }

  @Override
  public void clearSolidColor() {
    currentColor = null;
  }
}
