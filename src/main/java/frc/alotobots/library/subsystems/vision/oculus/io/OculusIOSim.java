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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Random;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/** Simulation implementation of OculusIO that provides realistic noisy measurements. */
public class OculusIOSim implements OculusIO {
  private final Random random = new Random();
  private final SwerveDriveSimulation swerveDriveSimulation;
  private double simulationTimeSeconds = 0.0;
  private static final double UPDATE_PERIOD_SECONDS = 1.0 / 120.0; // 120Hz update rate

  // Current simulated pose with noise
  private Pose2d currentPose = new Pose2d();

  // Transform representing the offset between where Quest thinks it is vs actual position
  private Transform2d poseOffset = new Transform2d();

  // Tracking simulation
  private boolean isCurrentlyTracking = true;
  private int trackingLostCounter = 0;
  private double lastTrackingLossTime = 0.0;
  private final double TRACKING_LOSS_INTERVAL =
      30.0; // Lose tracking every 30 seconds for simulation

  // Connection simulation
  private double lastUpdateTime = 0.0;

  public OculusIOSim(SwerveDriveSimulation swerveDriveSimulation) {
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.lastUpdateTime = Timer.getTimestamp();
    this.lastTrackingLossTime = Timer.getTimestamp();
  }

  /** Updates the base physics simulation pose that the Oculus measurements will be derived from. */
  private void updateSimPose() {
    // Get actual robot pose from physics
    Pose2d physicsPose = swerveDriveSimulation.getSimulatedDriveTrainPose();

    // Apply the stored offset from any imperfect resets
    Pose2d offsetPose = physicsPose.transformBy(poseOffset);

    // Transform robot pose to headset pose (assuming headset is at robot center for simplicity)
    currentPose = offsetPose;

    // Add noise based on standard deviations
    double noiseX = random.nextGaussian() * (OCULUS_STD_DEVS.get(0, 0) / SIM_TRUST_TRANSLATION);
    double noiseY = random.nextGaussian() * (OCULUS_STD_DEVS.get(1, 0) / SIM_TRUST_TRANSLATION);
    double noiseRot = random.nextGaussian() * (OCULUS_STD_DEVS.get(2, 0) / SIM_TRUST_ROTATION);

    currentPose =
        new Pose2d(
            currentPose.getX() + noiseX,
            currentPose.getY() + noiseY,
            currentPose.getRotation().plus(new Rotation2d(noiseRot)));
  }

  /** Simulates periodic tracking loss for realistic behavior */
  private void updateTrackingSimulation() {
    double currentTime = Timer.getTimestamp();

    // Simulate tracking loss every so often
    if (isCurrentlyTracking && (currentTime - lastTrackingLossTime) > TRACKING_LOSS_INTERVAL) {
      isCurrentlyTracking = false;
      trackingLostCounter++;
      lastTrackingLossTime = currentTime;
      Logger.recordOutput("Oculus/Log", "Simulated tracking loss event #" + trackingLostCounter);
    }

    // Regain tracking after 2 seconds
    if (!isCurrentlyTracking && (currentTime - lastTrackingLossTime) > 2.0) {
      isCurrentlyTracking = true;
      Logger.recordOutput("Oculus/Log", "Simulated tracking regained");
    }
  }

  @Override
  public void updateInputs(OculusIOInputs inputs) {
    // Update simulation time and pose
    simulationTimeSeconds += UPDATE_PERIOD_SECONDS;
    updateSimPose();
    updateTrackingSimulation();

    // Update current time for connection simulation
    lastUpdateTime = Timer.getTimestamp();

    // Simulate connection status (always connected in sim unless something goes wrong)
    inputs.connected = true;

    // Frame data
    inputs.frameCount = (int) (simulationTimeSeconds * 120); // 120Hz frame count
    inputs.timestamp = simulationTimeSeconds;

    // Latency
    inputs.latency = 20;
    // Pose data - if not tracking, provide stale data
    if (isCurrentlyTracking) {
      inputs.pose2d = currentPose;
    } else {
      // When not tracking, pose doesn't update (stale data)
      // inputs.pose2d retains its previous value
    }

    // Device data
    inputs.batteryPercent =
        Math.max(95.0, 100.0 - (simulationTimeSeconds * 0.01)); // Slowly drain battery
    inputs.currentlyTracking = isCurrentlyTracking;
    inputs.trackingLostCounter = trackingLostCounter;
  }

  @Override
  public void setPose(Pose2d oculusTargetPose) {
    // Store the offset between where we're telling Quest it is vs where it actually is
    Pose2d actualPose = swerveDriveSimulation.getSimulatedDriveTrainPose();
    poseOffset = new Transform2d(actualPose, oculusTargetPose);

    Logger.recordOutput(
        "Oculus/Log",
        String.format(
            "Pose reset in sim to: (%.2f, %.2f, %.2f°)",
            oculusTargetPose.getX(),
            oculusTargetPose.getY(),
            oculusTargetPose.getRotation().getDegrees()));
  }
}
