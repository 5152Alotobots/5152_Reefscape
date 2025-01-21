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

import static frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants;
import java.util.Random;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/** Simulation implementation of OculusIO that provides realistic noisy measurements. */
public class OculusIOSim implements OculusIO {
  private final Random random = new Random();
  private final SwerveDriveSimulation swerveDriveSimulation;
  private double simulationTimeSeconds = 0.0;
  private static final double UPDATE_PERIOD_SECONDS = 1.0 / 120.0; // 120Hz update rate

  // Current MOSI command state
  private int currentMosiValue = 0;
  private int currentMisoValue = 0;

  // Target pose for reset operations
  private Pose2d resetTargetPose = new Pose2d();

  // Current simulated pose with noise
  private Pose2d currentPose = new Pose2d();

  // Transform representing the offset between where Quest thinks it is vs actual position
  private Transform2d poseOffset = new Transform2d();

  public OculusIOSim(SwerveDriveSimulation swerveDriveSimulation) {
    this.swerveDriveSimulation = swerveDriveSimulation;
  }

  /** Updates the base physics simulation pose that the Oculus measurements will be derived from. */
  public void updateSimPose() {
    // Get actual robot pose from physics
    Pose2d physicsPose = swerveDriveSimulation.getSimulatedDriveTrainPose();

    // Apply the stored offset from any imperfect resets before transforming to headset position
    Pose2d offsetPose = physicsPose.transformBy(poseOffset);

    // Transform robot pose to headset pose
    // This simulates where the headset actually is relative to robot center
    // The subsystem will later apply the inverse of this transform to get back to robot pose
    currentPose = offsetPose; // .transformBy(OculusConstants.ROBOT_TO_OCULUS.inverse());

    // Add noise based on standard deviations
    double noiseX =
        random.nextGaussian() * (OculusConstants.OCULUS_STD_DEVS.get(0, 0) / SIM_TRUST_TRANSLATION);
    double noiseY =
        random.nextGaussian() * (OculusConstants.OCULUS_STD_DEVS.get(1, 0) / SIM_TRUST_TRANSLATION);
    double noiseRot =
        random.nextGaussian() * (OculusConstants.OCULUS_STD_DEVS.get(2, 0) / SIM_TRUST_ROTATION);

    currentPose =
        new Pose2d(
            currentPose.getX() + noiseX,
            currentPose.getY() + noiseY,
            currentPose.getRotation().plus(new Rotation2d(noiseRot)));
  }

  @Override
  public void updateInputs(OculusIOInputs inputs) {
    // Update quest pose
    updateSimPose();

    // Update simulation time
    simulationTimeSeconds += UPDATE_PERIOD_SECONDS;

    // Convert pose to Quest coordinate system (Z forward, -X left)
    // Quest coordinates: +Z forward, +Y up, -X left
    // FRC coordinates: +X forward, +Y left
    float[] position =
        new float[] {
          (float) -currentPose.getY(), // Quest -X = FRC Y
          0.0f, // Quest Y (height) = 0
          (float) currentPose.getX() // Quest Z = FRC X
        };

    // Convert rotation to Quest coordinate system
    float yaw = (float) -currentPose.getRotation().getDegrees();
    float[] eulerAngles =
        new float[] {
          0.0f, // Roll
          yaw, // Yaw (negated for coordinate system conversion)
          0.0f // Pitch
        };

    // Convert to quaternion using WPILib
    // The Quaternion constructor takes (w, x, y, z)
    // For a pure yaw rotation:
    // w = cos(angle/2)
    // z = sin(angle/2)
    double halfAngle = currentPose.getRotation().getRadians() / 2.0;
    var wpilibQuaternion =
        new Quaternion(
            Math.cos(halfAngle), // w
            0.0, // x
            0.0, // y
            Math.sin(halfAngle) // z
            );

    float[] quaternion =
        new float[] {
          (float) wpilibQuaternion.getW(),
          (float) wpilibQuaternion.getX(),
          (float) wpilibQuaternion.getY(),
          (float) wpilibQuaternion.getZ()
        };

    // Update input values
    inputs.position = position;
    inputs.eulerAngles = eulerAngles;
    inputs.quaternion = quaternion;
    inputs.timestamp = simulationTimeSeconds;
    inputs.frameCount = (int) (simulationTimeSeconds * 120); // 120Hz frame count
    inputs.batteryPercent = 100.0;

    // Handle MISO responses based on MOSI commands
    handleMisoResponse(inputs);
  }

  private void handleMisoResponse(OculusIOInputs inputs) {
    // Update MISO value based on current command state
    switch (currentMosiValue) {
      case 0: // No command
        currentMisoValue = 0; // STATUS_READY
        break;
      case 1: // Heading reset
        currentMisoValue = 99; // STATUS_HEADING_RESET_COMPLETE
        break;
      case 2: // Pose reset
        // Store the offset between where we're telling Quest it is vs where it actually is
        Pose2d actualPose = swerveDriveSimulation.getSimulatedDriveTrainPose();
        poseOffset = new Transform2d(actualPose, resetTargetPose);
        currentMisoValue = 98; // STATUS_POSE_RESET_COMPLETE
        break;
      case 3: // Ping
        currentMisoValue = 97; // STATUS_PING_RESPONSE
        break;
    }

    inputs.misoValue = currentMisoValue;
  }

  @Override
  public void setMosi(int value) {
    this.currentMosiValue = value;
  }

  @Override
  public void setResetPose(double x, double y, double rotation) {
    this.resetTargetPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
  }
}
