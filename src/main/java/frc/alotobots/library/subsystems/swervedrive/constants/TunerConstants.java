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
package frc.alotobots.library.subsystems.swervedrive.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * Interface defining constants and configuration values for swerve drive tuning. This interface
 * provides access to module-specific configurations, drivetrain constants, and various control
 * parameters needed for swerve drive operation.
 */
public interface TunerConstants {
  /**
   * Gets the configuration constants for the front left swerve module.
   *
   * @return SwerveModuleConstants containing TalonFX and CANcoder configurations for the front left
   *     module
   */
  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getFrontLeft();

  /**
   * Gets the configuration constants for the front right swerve module.
   *
   * @return SwerveModuleConstants containing TalonFX and CANcoder configurations for the front
   *     right module
   */
  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getFrontRight();

  /**
   * Gets the configuration constants for the back left swerve module.
   *
   * @return SwerveModuleConstants containing TalonFX and CANcoder configurations for the back left
   *     module
   */
  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getBackLeft();

  /**
   * Gets the configuration constants for the back right swerve module.
   *
   * @return SwerveModuleConstants containing TalonFX and CANcoder configurations for the back right
   *     module
   */
  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getBackRight();

  /**
   * Gets the general drivetrain constants for the swerve drive.
   *
   * @return SwerveDrivetrainConstants containing overall drivetrain configuration
   */
  SwerveDrivetrainConstants getDrivetrainConstants();

  /**
   * Gets the radius of the drive base from center to furthest module.
   *
   * @return The drive base radius in meters
   */
  double getDriveBaseRadius();

  /**
   * Gets the robot's maximum speed at 12 volts.
   *
   * @return The maximum speed as a LinearVelocity
   */
  LinearVelocity getSpeedAt12Volts();

  /**
   * Gets the robot's reduced speed for precision control.
   *
   * @return The turtle mode speed as a LinearVelocity
   */
  LinearVelocity getTurtleSpeed();

  /**
   * Gets the robot's standard operating speed.
   *
   * @return The nominal speed as a LinearVelocity
   */
  LinearVelocity getNominalSpeed();

  /**
   * Gets the robot's maximum boosted speed.
   *
   * @return The turbo mode speed as a LinearVelocity
   */
  LinearVelocity getTurboSpeed();

  /**
   * Gets the maximum rotational rate of individual modules.
   *
   * @return The maximum rotational rate in radians per second
   */
  double getMaxModularRotationalRate();

  /**
   * Gets the frequency at which odometry updates are processed.
   *
   * @return The odometry update frequency in hertz
   */
  double getOdometryFrequency();

  /**
   * Gets the length of the robot with bumpers.
   *
   * @return The bumper length as a Distance
   */
  Distance getBumperLength();

  /**
   * Gets the width of the robot with bumpers.
   *
   * @return The bumper width as a Distance
   */
  Distance getBumperWidth();

  /**
   * Gets the PathPlanner configuration for the robot.
   *
   * @return RobotConfig containing PathPlanner-specific settings
   */
  RobotConfig getPathPlannerConfig();

  /**
   * Gets the constraints for PathPlanner pathfinding.
   *
   * @return PathConstraints containing velocity and acceleration limits
   */
  PathConstraints getPathfindingConstraints();

  /**
   * Gets the holonomic drive controller for path following.
   *
   * @return PPHolonomicDriveController for path following control
   */
  PPHolonomicDriveController getHolonomicDriveController();

  /**
   * Gets the PID gains for the steer motors.
   *
   * @return Slot0Configs containing PID and feedforward gains for steering
   */
  Slot0Configs getSteerGains();

  /**
   * Gets the PID gains for the drive motors.
   *
   * @return Slot0Configs containing PID and feedforward gains for driving
   */
  Slot0Configs getDriveGains();

  /**
   * Gets the physical positions of all swerve modules relative to the robot center.
   *
   * @return Array of Translation2d representing module positions
   */
  Translation2d[] getModuleTranslations();

  /**
   * Gets the PID controller for maintaining robot heading while driving.
   *
   * @return ProfiledPIDController for heading control
   */
  ProfiledPIDController getDriveFacingAnglePIDController();

  /**
   * Gets the precision align tolerance (when it stops trying to correct)
   *
   * @return double of tolerance
   */
  double getPrecisionAlignTolerance();

  /**
   * Gets the radius in which precision align should function
   *
   * @return double of tolerance
   */
  double getPrecisionAlignAllowRadius();

  /**
   * Gets the simulation config for Maple-Sim
   *
   * @return The simulation config
   */
  DriveTrainSimulationConfig getDriveTrainSimulationConfig();
}
