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

    /** 3D position coordinates [x, y, z] */
    public float[] position = new float[] {0.0f, 0.0f, 0.0f};

    /** Quaternion orientation [w, x, y, z] */
    public float[] quaternion = new float[] {0.0f, 0.0f, 0.0f, 0.0f};

    /** Euler angles [roll, pitch, yaw] in degrees */
    public float[] eulerAngles = new float[] {0.0f, 0.0f, 0.0f};

    /** Current timestamp from the Oculus */
    public double timestamp = -1.0;

    /** Frame counter from the Oculus */
    public int frameCount = -1;

    /** Battery level percentage */
    public double batteryPercent = -1.0;

    /** Current MISO (Master In Slave Out) value */
    public int misoValue = 0;

    /** Does the Oculus have 6dof tracking? */
    public boolean isTracking = false;

    /** Total number of tracking lost events since the Quest has booted */
    public int totalTrackingLostEvents = 0;
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
  public default void resetPose(Pose2d oculusTargetPose) {}

  /** Resets the current heading of the Oculus as zero */
  public default void resetHeading() {}
}
