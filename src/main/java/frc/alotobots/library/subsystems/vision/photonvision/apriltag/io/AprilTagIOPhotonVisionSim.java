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
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.io;

import static frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants.APRIL_TAG_LAYOUT;

import edu.wpi.first.math.geometry.Pose2d;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.CameraConfig;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagIOPhotonVisionSim extends AprilTagIOPhotonVision {
  private static VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  public AprilTagIOPhotonVisionSim(CameraConfig config, Supplier<Pose2d> poseSupplier) {
    super(config);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(APRIL_TAG_LAYOUT);
    }

    // Add sim camera
    cameraSim = new PhotonCameraSim(camera, config.properties());
    visionSim.addCamera(cameraSim, config.robotToCamera());
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
