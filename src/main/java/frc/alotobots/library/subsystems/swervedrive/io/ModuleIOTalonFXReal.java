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
package frc.alotobots.library.subsystems.swervedrive.io;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.PhoenixOdometryThread;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
  // Queue to read inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOTalonFXReal(int moduleIndex) {
    super(
        switch (moduleIndex) {
          case 0 -> Constants.tunerConstants.getFrontLeft();
          case 1 -> Constants.tunerConstants.getFrontRight();
          case 2 -> Constants.tunerConstants.getBackLeft();
          case 3 -> Constants.tunerConstants.getBackRight();
          default -> throw new IllegalArgumentException("Invalid module index: " + moduleIndex);
        });

    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
    this.turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.turnAbsolutePosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);
    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }
}
