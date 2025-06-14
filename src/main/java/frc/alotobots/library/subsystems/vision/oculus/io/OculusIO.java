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
package frc.alotobots.library.subsystems.vision.oculus.io;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for handling input/output operations with the Oculus Quest hardware. */
public interface OculusIO {
  /** Data structure for Oculus inputs that can be automatically logged. */
  @AutoLog
  public static class OculusIOInputs {

    public boolean connected = false;

    /** Frame counter from the Oculus */
    public int frameCount = -1;

    /** Current timestamp from the Oculus */
    public double timestamp = -1.0;

    /** Current pose from the Oculus */
    public Pose2d pose2d = Pose2d.kZero;

    /** Total number of tracking lost events since the Quest has booted */
    public int trackingLostCounter = 0;

    /** Does the Oculus have 6dof tracking? */
    public boolean currentlyTracking = false;

    /** Battery level percentage */
    public double batteryPercent = -1.0;

    /** Quest > Robot Latency in MS */
    public double latency = -1.0;
  }

  /**
   * Updates the set of loggable inputs from the Oculus.
   *
   * @param inputs The input object to update with current values
   */
  public default void updateInputs(OculusIOInputs inputs) {}

  /**
   * Resets the pose components for resetting the Oculus position tracking. HARD RESET.
   *
   * @param oculusTargetPose The target pose of the oculus to reset to. NOT THE TARGET ROBOT POSE
   */
  public default void setPose(Pose2d oculusTargetPose) {}
}
