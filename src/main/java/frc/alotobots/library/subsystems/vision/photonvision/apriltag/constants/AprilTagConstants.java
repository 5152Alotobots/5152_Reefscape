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
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.experimental.UtilityClass;
import org.photonvision.simulation.SimCameraProperties;

@SuppressWarnings("resource")
@UtilityClass
public class AprilTagConstants {
  // AprilTag layout
  public static AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // OFFSETS
  // FORWARD: +, LEFT: +, UP: + (USED FOR APRILTAGS)
  private static final Transform3d[] CAMERA_OFFSETS =
      new Transform3d[] {
        // Front Left
        new Transform3d(
            new Translation3d(0.245, 0.21, 0.17),
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(45))),
        // Front Middle
        new Transform3d(
            new Translation3d(0.275, 0.0, 0.189),
            new Rotation3d(0, Math.toRadians(-35), Math.toRadians(0)))
      };

  // CAMERAS
  public static final CameraConfig[] CAMERA_CONFIGS = {
    new CameraConfig("FL_AprilTag", CAMERA_OFFSETS[0], new SimCameraProperties()),
    new CameraConfig("FM_AprilTag", CAMERA_OFFSETS[1], new SimCameraProperties())
  };

  /*
   * The standard deviations to use for the vision
   */
  // NEEDS TO BE COPIED FROM PHOTONVISION ON COMPETITION DAY!!
  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  // LOWER = TRUST MORE
  public static double LINEAR_STD_DEV_BASE = 0.02; // Meters
  public static double ANGULAR_STD_DEV_BASE = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // SHOULD NEVER BE LESS THAN 1.0, NUMBERS GREATER THAN 1 = TRUST LESS
  public static double[] CAMERA_STD_DEV_FACTORS = new double[] {1.0, 1.0};

  // Basic Filtering
  public static double MAX_AMBIGUITY = 0.3;
  public static double MAX_Z_ERROR = 0.75;

  /** Time after which a pose is considered stale (seconds) */
  public static final double POSE_TIMEOUT = 0.06;
}
