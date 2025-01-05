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
package frc.alotobots.library.subsystems.swervedrive.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface TunerConstants {
  // Module Constants
  SwerveModuleConstants getFrontLeft();

  SwerveModuleConstants getFrontRight();

  SwerveModuleConstants getBackLeft();

  SwerveModuleConstants getBackRight();

  // Drivetrain Constants
  SwerveDrivetrainConstants getDrivetrainConstants();

  double getDriveBaseRadius();

  LinearVelocity getSpeedAt12Volts();

  LinearVelocity getTurtleSpeed();

  LinearVelocity getNominalSpeed();

  LinearVelocity getTurboSpeed();

  double getMaxModularRotationalRate();

  double getOdometryFrequency();

  // Physical Dimensions
  Distance getBumperLength();

  Distance getBumperWidth();

  // PathPlanner Integration
  RobotConfig getPathPlannerConfig();

  PathConstraints getPathfindingConstraints();

  PPHolonomicDriveController getHolonomicDriveController();

  // Control Gains
  Slot0Configs getSteerGains();

  Slot0Configs getDriveGains();

  // Module Translations
  Translation2d[] getModuleTranslations();

  // PIDs
  ProfiledPIDController getDriveFacingAnglePIDController();
}
