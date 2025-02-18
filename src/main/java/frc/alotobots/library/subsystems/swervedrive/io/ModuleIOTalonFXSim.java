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

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import frc.alotobots.Constants;
import frc.alotobots.util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(int moduleIndex, SwerveModuleSimulation simulation) {
    super(
        switch (moduleIndex) {
          case 0 -> Constants.tunerConstants.getFrontLeft();
          case 1 -> Constants.tunerConstants.getFrontRight();
          case 2 -> Constants.tunerConstants.getBackLeft();
          case 3 -> Constants.tunerConstants.getBackRight();
          default -> throw new IllegalArgumentException("Invalid module index: " + moduleIndex);
        });

    this.simulation = simulation;
    simulation.useDriveMotorController(
        new PhoenixUtil.TalonFXMotorControllerSim(driveTalon, constants.DriveMotorInverted));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(
            turnTalon,
            constants.SteerMotorInverted,
            cancoder,
            constants.EncoderInverted,
            Rotations.of(constants.EncoderOffset)));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();

    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }
}
