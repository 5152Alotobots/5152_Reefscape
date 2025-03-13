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
package frc.alotobots.library.subsystems.vision.photonvision.apriltag.io;

import static frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants.APRIL_TAG_LAYOUT;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.CameraConfig;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class AprilTagIOPhotonVision implements AprilTagIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final Supplier<Rotation2d> yawSupplier;

  private final TimeInterpolatableBuffer<Rotation2d> singleTagHeadingBuffer =
      TimeInterpolatableBuffer.createBuffer(1.0);

  /**
   * Constructor for the AprilTagIOPhotonVision class.
   *
   * @param config The camera configuration object containing camera properties.
   * @param yawSupplier A supplier for the yaw angle of the robot in field-relative space
   */
  public AprilTagIOPhotonVision(CameraConfig config, Supplier<Rotation2d> yawSupplier) {
    camera = new PhotonCamera(config.name());
    this.robotToCamera = config.robotToCamera();
    this.yawSupplier = yawSupplier;
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    singleTagHeadingBuffer.addSample(Timer.getTimestamp(), yawSupplier.get());
    inputs.connected = camera.isConnected();
    inputs.singleTagHeadingBuffer =
        singleTagHeadingBuffer
            .getInternalBuffer()
            .values()
            .toArray(
                new Rotation2d
                    [singleTagHeadingBuffer.getInternalBuffer().values().toArray().length]);

    // Read new camera observations
    Set<Short> multiTagIds = new HashSet<>();
    Set<Integer> singleTagIds = new HashSet<>();
    List<MultiTagObservation> multiTagObservations = new LinkedList<>();
    List<SingleTagObservation> singleTagObservations = new LinkedList<>();

    for (var result : camera.getAllUnreadResults()) {
      processMultiTagObservations(result, multiTagIds, multiTagObservations);
      processSingleTagObservations(result, singleTagIds, singleTagObservations);
    }

    // Save pose observations to inputs object
    inputs.singleTagObservations = new SingleTagObservation[singleTagObservations.size()];
    for (int i = 0; i < singleTagObservations.size(); i++) {
      inputs.singleTagObservations[i] = singleTagObservations.get(i);
    }

    inputs.multiTagObservations = new MultiTagObservation[multiTagObservations.size()];
    for (int i = 0; i < multiTagObservations.size(); i++) {
      inputs.multiTagObservations[i] = multiTagObservations.get(i);
    }

    // Save multi tag IDs to inputs objects
    inputs.multiTagIds = new int[multiTagIds.size()];
    int i = 0;
    for (int id : multiTagIds) {
      inputs.multiTagIds[i++] = id;
    }

    // Save single tag IDs to inputs objects
    inputs.singleTagIds = new int[singleTagIds.size()];
    int j = 0;
    for (int id : singleTagIds) {
      inputs.singleTagIds[j++] = id;
    }
  }

  private void processMultiTagObservations(
      PhotonPipelineResult result,
      Set<Short> multiTagIds,
      List<MultiTagObservation> multiTagObservations) {
    if (result.multitagResult.isPresent()) {
      var multitagResult = result.multitagResult.get();

      // Calculate robot pose
      Transform3d fieldToCamera = multitagResult.estimatedPose.best;
      Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

      // Calculate average tag distance
      double totalTagDistance = 0.0;
      for (var target : result.targets) {
        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
      }

      // Add tag IDs
      multiTagIds.addAll(multitagResult.fiducialIDsUsed);

      // Add observation
      multiTagObservations.add(
          new MultiTagObservation(
              result.getTimestampSeconds(), // Timestamp
              robotPose, // 3D pose estimate
              multitagResult.estimatedPose.ambiguity, // Ambiguity
              multitagResult.fiducialIDsUsed.size(), // Tag count
              totalTagDistance / result.targets.size()));
    }
  }

  private void processSingleTagObservations(
      PhotonPipelineResult result,
      Set<Integer> singleTagIds,
      List<SingleTagObservation> singleTagObservations) {
    if (result.hasTargets()) {

      var bestTarget = result.getBestTarget();

      Translation2d cameraToTagTranslation =
          new Pose3d(
                  Translation3d.kZero,
                  new Rotation3d(
                      0,
                      -Math.toRadians(bestTarget.getPitch()),
                      -Math.toRadians(bestTarget.getYaw())))
              .transformBy(
                  new Transform3d(
                      new Translation3d(
                          bestTarget.getBestCameraToTarget().getTranslation().getNorm(), 0, 0),
                      Rotation3d.kZero))
              .getTranslation()
              .rotateBy(
                  new Rotation3d(
                      robotToCamera.getRotation().getX(), robotToCamera.getRotation().getY(), 0))
              .toTranslation2d();

      if (singleTagHeadingBuffer.getSample(result.getTimestampSeconds()).isPresent()) {
        Rotation2d headingSample =
            singleTagHeadingBuffer.getSample(result.getTimestampSeconds()).get();

        Rotation2d cameraToTagRotation =
            headingSample.plus(
                robotToCamera.getRotation().toRotation2d().plus(cameraToTagTranslation.getAngle()));

        if (APRIL_TAG_LAYOUT.getTagPose(bestTarget.getFiducialId()).isPresent()) {
          Pose2d tagPose = APRIL_TAG_LAYOUT.getTagPose(bestTarget.getFiducialId()).get().toPose2d();

          Translation2d fieldToCameraTranslation =
              new Pose2d(tagPose.getTranslation(), cameraToTagRotation.plus(Rotation2d.kPi))
                  .transformBy(
                      new Transform2d(cameraToTagTranslation.getNorm(), 0, Rotation2d.kZero))
                  .getTranslation();

          Pose2d robotPose =
              new Pose2d(
                      fieldToCameraTranslation,
                      headingSample.plus(robotToCamera.getRotation().toRotation2d()))
                  .transformBy(
                      new Transform2d(
                          new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation())
                              .toPose2d(),
                          Pose2d.kZero));

          robotPose = new Pose2d(robotPose.getTranslation(), headingSample);

          singleTagIds.add(bestTarget.getFiducialId());

          singleTagObservations.add(
              new SingleTagObservation(
                  result.getTimestampSeconds(),
                  robotPose,
                  bestTarget.poseAmbiguity,
                  cameraToTagTranslation.getNorm()));
        }
      }
    }
  }
}
