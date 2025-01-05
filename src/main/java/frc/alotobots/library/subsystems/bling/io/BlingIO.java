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

import com.ctre.phoenix.led.Animation;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for controlling LED lighting functionality on the robot. Provides methods for setting
 * animations, solid colors, and managing LED states.
 */
public interface BlingIO {
  /** Data structure for logging LED state information. */
  @AutoLog
  public static class BlingIOInputs {
    /** Current solid color setting of the LEDs */
    public LoggedColor currentSolidColor;

    /** Flag indicating if an animation is currently active */
    public boolean hasAnimation = false;

    /** Flag indicating if a solid color is currently set */
    public boolean hasColor = false;

    /** Name of the currently running animation */
    public String animationName = "";
  }

  /**
   * Record class representing an RGB color value for LED control.
   *
   * @param red Red component value (0-255)
   * @param green Green component value (0-255)
   * @param blue Blue component value (0-255)
   */
  public static record LoggedColor(int red, int green, int blue) {}

  /**
   * Updates the input values for logging and monitoring.
   *
   * @param inputs The inputs object to update with current state
   */
  public default void updateInputs(BlingIOInputs inputs) {}

  /**
   * Sets a new animation pattern for the LEDs.
   *
   * @param animation The animation pattern to display
   */
  public default void setAnimation(Animation animation) {}

  /** Clears the current animation, stopping any running patterns. */
  public default void clearAnimation() {}

  /**
   * Sets the LEDs to display a solid color.
   *
   * @param color The color to display
   */
  public default void setSolidColor(BlingIO.LoggedColor color) {}

  /** Clears the current solid color, turning off all LEDs. */
  public default void clearSolidColor() {}
}
