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

import static edu.wpi.first.units.Units.Milliseconds;
import static frc.alotobots.library.subsystems.vision.oculus.constants.OculusConstants.*;
import static frc.alotobots.library.subsystems.vision.oculus.util.OculusStatus.Miso.*;
import static frc.alotobots.library.subsystems.vision.oculus.util.OculusStatus.Mosi.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
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

  // Current command state
  private int currentMosiValue = COMMAND_CLEAR;
  private int currentMisoValue = STATUS_READY;

  // Target pose for reset operations
  private Pose2d resetTargetPose = new Pose2d();

  // Current simulated pose with noise
  private Pose2d currentPose = new Pose2d();

  // Transform representing the offset between where Quest thinks it is vs actual position
  private Transform2d poseOffset = new Transform2d();

  // Heartbeat simulation
  private double lastHeartbeatTime = 0.0;
  private double lastProcessedHeartbeatId = 0.0;

  public OculusIOSim(SwerveDriveSimulation swerveDriveSimulation) {
    this.swerveDriveSimulation = swerveDriveSimulation;
    this.lastHeartbeatTime = Timer.getTimestamp();
  }

  /** Updates the base physics simulation pose that the Oculus measurements will be derived from. */
  private void updateSimPose() {
    // Get actual robot pose from physics
    Pose2d physicsPose = swerveDriveSimulation.getSimulatedDriveTrainPose();

    // Apply the stored offset from any imperfect resets before transforming to headset position
    Pose2d offsetPose = physicsPose.transformBy(poseOffset);

    // Transform robot pose to headset pose
    // This simulates where the headset actually is relative to robot center
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

    // Simulate a connected device by checking the time since the last heartbeat
    inputs.connected =
        Milliseconds.of(Timer.getTimestamp() - lastHeartbeatTime).lt(OCULUS_CONNECTION_TIMEOUT);
    inputs.position = position;
    inputs.eulerAngles = eulerAngles;
    inputs.quaternion = quaternion;
    inputs.timestamp = simulationTimeSeconds;
    inputs.frameCount = (int) (simulationTimeSeconds * 120); // 120Hz frame count
    inputs.batteryPercent = 100.0;
    inputs.misoValue = currentMisoValue;

    // Process any pending commands
    processCommands();

    // Simulate heartbeat every second
    if (Timer.getTimestamp() - lastHeartbeatTime > 1.0) {
      lastHeartbeatTime = Timer.getTimestamp();
      lastProcessedHeartbeatId += 1.0;
    }
  }

  private void processCommands() {
    // Process any pending commands based on current MOSI value
    switch (currentMosiValue) {
      case COMMAND_RESET_HEADING -> {
        Logger.recordOutput("Oculus/Log", "Heading reset command received in sim");
        currentMisoValue = STATUS_HEADING_RESET_COMPLETE;
        currentMosiValue = COMMAND_CLEAR; // Auto-clear the command
      }

      case COMMAND_RESET_POSE -> {
        Logger.recordOutput("Oculus/Log", "Pose reset command received in sim");
        // Store the offset between where we're telling Quest it is vs where it actually is
        Pose2d actualPose = swerveDriveSimulation.getSimulatedDriveTrainPose();
        poseOffset = new Transform2d(actualPose, resetTargetPose);
        currentMisoValue = STATUS_POSE_RESET_COMPLETE;
        currentMosiValue = COMMAND_CLEAR; // Auto-clear the command
      }

      case COMMAND_PING -> {
        Logger.recordOutput("Oculus/Log", "Ping command received in sim");
        currentMisoValue = STATUS_PING_RESPONSE;
        currentMosiValue = COMMAND_CLEAR; // Auto-clear the command
      }

      case COMMAND_CLEAR -> {
        if (currentMisoValue != STATUS_READY) {
          currentMisoValue = STATUS_READY;
        }
      }
    }
  }

  @Override
  public void resetPose(double x, double y, double rotation) {
    resetTargetPose = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));

    // Force clear mosi
    currentMosiValue = COMMAND_CLEAR;

    // Send reset command
    currentMosiValue = COMMAND_RESET_POSE;
  }

  @Override
  public void resetHeading() {
    // Force clear mosi
    currentMosiValue = COMMAND_CLEAR;

    // Send reset command
    currentMosiValue = COMMAND_RESET_HEADING;
  }
}
