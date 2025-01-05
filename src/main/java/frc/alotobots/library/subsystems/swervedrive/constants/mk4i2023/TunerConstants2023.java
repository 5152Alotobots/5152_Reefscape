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
package frc.alotobots.library.subsystems.swervedrive.constants.mk4i2023;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.constants.TunerConstants;

public class TunerConstants2023 implements TunerConstants {
  public static class GeneratedConstants {
    // Copied directly from the original generated constants file
    public static final Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(55)
            .withKI(0)
            .withKD(0.2)
            .withKS(0.12)
            .withKV(0.102)
            .withKA(0.015)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    public static final Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(.05)
            .withKI(0)
            .withKD(0.002)
            .withKS(0.19437)
            .withKV(0.75843)
            .withKA(0.01);
    public static final SwerveModuleConstants.ClosedLoopOutputType kSteerClosedLoopOutput =
        SwerveModuleConstants.ClosedLoopOutputType.Voltage;
    public static final SwerveModuleConstants.ClosedLoopOutputType kDriveClosedLoopOutput =
        ClosedLoopOutputType.Voltage;
    public static final Current kSlipCurrent = Amps.of(27.16);
    public static final double kMaxModularRotationalRate = Units.rotationsToRadians(12);
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.02);
    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true));
    public static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    public static final Pigeon2Configuration pigeonConfigs = null;
    public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");
    public static final double ODOMETRY_FREQUENCY = kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double kCoupleRatio = 3.5714285714285716;
    public static final double kDriveGearRatio = 6.746031746031747;
    public static final double kSteerGearRatio = 21.428571428571427;
    public static final Distance kWheelRadius = Inches.of(2);
    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;
    public static final boolean kSteerMotorInverted = true;
    public static final boolean kCanCoderInverted = false;
    public static final double kSteerInertia = 0.00001;
    public static final double kDriveInertia = 0.001;
    public static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
    public static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withPigeon2Id(Constants.CanId.PIGEON_2_ID)
            .withCANBusName(kCANBus.getName())
            .withPigeon2Configs(pigeonConfigs);

    public static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadius)
            .withSlipCurrent(kSlipCurrent)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withCANcoderInitialConfigs(cancoderInitialConfigs);

    public static final int kFrontLeftDriveMotorId = Constants.CanId.FRONT_LEFT_DRIVE_MTR_CAN_ID;
    public static final int kFrontLeftSteerMotorId = Constants.CanId.FRONT_LEFT_STEER_MTR_CAN_ID;
    public static final int kFrontLeftEncoderId = Constants.CanId.FRONT_LEFT_STEER_CAN_CODER_CAN_ID;
    public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.491943359375);
    public static final Distance kFrontLeftXPos = Inches.of(9.25);
    public static final Distance kFrontLeftYPos = Inches.of(9.25);

    public static final int kFrontRightDriveMotorId = Constants.CanId.FRONT_RIGHT_DRIVE_MTR_CAN_ID;
    public static final int kFrontRightSteerMotorId = Constants.CanId.FRONT_RIGHT_STEER_MTR_CAN_ID;
    public static final int kFrontRightEncoderId =
        Constants.CanId.FRONT_RIGHT_STEER_CAN_CODER_CAN_ID;
    public static final Angle kFrontRightEncoderOffset = Rotations.of(0.1962890625);
    public static final Distance kFrontRightXPos = Inches.of(9.25);
    public static final Distance kFrontRightYPos = Inches.of(-9.25);

    public static final int kBackLeftDriveMotorId = Constants.CanId.BACK_LEFT_DRIVE_MTR_CAN_ID;
    public static final int kBackLeftSteerMotorId = Constants.CanId.BACK_LEFT_STEER_MTR_CAN_ID;
    public static final int kBackLeftEncoderId = Constants.CanId.BACK_LEFT_STEER_CAN_CODER_CAN_ID;
    public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.468017578125);
    public static final Distance kBackLeftXPos = Inches.of(-9.25);
    public static final Distance kBackLeftYPos = Inches.of(9.25);

    public static final int kBackRightDriveMotorId = Constants.CanId.BACK_RIGHT_DRIVE_MTR_CAN_ID;
    public static final int kBackRightSteerMotorId = Constants.CanId.BACK_RIGHT_STEER_MTR_CAN_ID;
    public static final int kBackRightEncoderId = Constants.CanId.BACK_RIGHT_STEER_CAN_CODER_CAN_ID;
    public static final Angle kBackRightEncoderOffset = Rotations.of(-0.2978515625);
    public static final Distance kBackRightXPos = Inches.of(-9.25);
    public static final Distance kBackRightYPos = Inches.of(-9.25);

    public static final SwerveModuleConstants FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            kFrontLeftXPos,
            kFrontLeftYPos,
            kInvertLeftSide,
            kSteerMotorInverted,
            kCanCoderInverted);

    public static final SwerveModuleConstants FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            kFrontRightXPos,
            kFrontRightYPos,
            kInvertRightSide,
            kSteerMotorInverted,
            kCanCoderInverted);

    public static final SwerveModuleConstants BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            kBackLeftXPos,
            kBackLeftYPos,
            kInvertLeftSide,
            kSteerMotorInverted,
            kCanCoderInverted);

    public static final SwerveModuleConstants BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            kBackRightXPos,
            kBackRightYPos,
            kInvertRightSide,
            kSteerMotorInverted,
            kCanCoderInverted);
  }

  public static class CustomConstants {
    // Custom constants go here
    public static final PIDConstants translationPid = new PIDConstants(2.4, 0, 0.015);
    public static final PIDConstants rotationPid = new PIDConstants(7.8, 0, 0.015);
    public static final PathConstraints PATHFINDING_CONSTRAINTS =
        new PathConstraints(5.02, 3.5, Units.degreesToRadians(540), Units.degreesToRadians(460));
    public static final PPHolonomicDriveController PP_HOLONOMIC_DRIVE_CONTROLLER =
        new PPHolonomicDriveController(translationPid, rotationPid);
    public static final ProfiledPIDController driveFacingAngleController =
        new ProfiledPIDController(
            rotationPid.kP,
            rotationPid.kI,
            rotationPid.kD,
            new TrapezoidProfile.Constraints(
                PATHFINDING_CONSTRAINTS.maxVelocityMPS(),
                PATHFINDING_CONSTRAINTS.maxAccelerationMPSSq()));
    public static final Distance BUMPER_LENGTH = Distance.ofBaseUnits(.75, Meters);
    public static final Distance BUMPER_WIDTH = Distance.ofBaseUnits(.75, Meters);
    public static final LinearVelocity kTurtleSpeed = MetersPerSecond.of(1.0);
    public static final LinearVelocity kNominalSpeed = MetersPerSecond.of(3.0);
    public static final LinearVelocity kTurboSpeed = MetersPerSecond.of(4.8);
    public static final double ROBOT_MASS_KG = 34;
    public static final double ROBOT_MOI = 2.550;
    public static final double WHEEL_COF = 0.6;

    public static final RobotConfig pathPlannerConfig =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                GeneratedConstants.FrontLeft.WheelRadius,
                GeneratedConstants.kSpeedAt12Volts.in(MetersPerSecond),
                WHEEL_COF,
                DCMotor.getFalcon500(1)
                    .withReduction(GeneratedConstants.FrontLeft.DriveMotorGearRatio),
                GeneratedConstants.FrontLeft.SlipCurrent,
                1),
            new Translation2d[] {
              new Translation2d(
                  GeneratedConstants.FrontLeft.LocationX, GeneratedConstants.FrontLeft.LocationY),
              new Translation2d(
                  GeneratedConstants.FrontRight.LocationX, GeneratedConstants.FrontRight.LocationY),
              new Translation2d(
                  GeneratedConstants.BackLeft.LocationX, GeneratedConstants.BackLeft.LocationY),
              new Translation2d(
                  GeneratedConstants.BackRight.LocationX, GeneratedConstants.BackRight.LocationY)
            });

    public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(
                    GeneratedConstants.FrontLeft.LocationX,
                    GeneratedConstants.FrontRight.LocationY),
                Math.hypot(
                    GeneratedConstants.FrontRight.LocationX,
                    GeneratedConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(
                    GeneratedConstants.BackLeft.LocationX, GeneratedConstants.BackLeft.LocationY),
                Math.hypot(
                    GeneratedConstants.BackRight.LocationX,
                    GeneratedConstants.BackRight.LocationY)));
  }

  @Override
  public SwerveModuleConstants getFrontLeft() {
    return GeneratedConstants.FrontLeft;
  }

  @Override
  public SwerveModuleConstants getFrontRight() {
    return GeneratedConstants.FrontRight;
  }

  @Override
  public SwerveModuleConstants getBackLeft() {
    return GeneratedConstants.BackLeft;
  }

  @Override
  public SwerveModuleConstants getBackRight() {
    return GeneratedConstants.BackRight;
  }

  @Override
  public SwerveDrivetrainConstants getDrivetrainConstants() {
    return GeneratedConstants.DrivetrainConstants;
  }

  @Override
  public PathConstraints getPathfindingConstraints() {
    return CustomConstants.PATHFINDING_CONSTRAINTS;
  }

  @Override
  public PPHolonomicDriveController getHolonomicDriveController() {
    return CustomConstants.PP_HOLONOMIC_DRIVE_CONTROLLER;
  }

  @Override
  public Distance getBumperLength() {
    return CustomConstants.BUMPER_LENGTH;
  }

  @Override
  public Distance getBumperWidth() {
    return CustomConstants.BUMPER_WIDTH;
  }

  @Override
  public double getDriveBaseRadius() {
    return CustomConstants.DRIVE_BASE_RADIUS;
  }

  @Override
  public double getOdometryFrequency() {
    return GeneratedConstants.ODOMETRY_FREQUENCY;
  }

  @Override
  public Slot0Configs getSteerGains() {
    return GeneratedConstants.steerGains;
  }

  @Override
  public Slot0Configs getDriveGains() {
    return GeneratedConstants.driveGains;
  }

  @Override
  public LinearVelocity getSpeedAt12Volts() {
    return GeneratedConstants.kSpeedAt12Volts;
  }

  @Override
  public LinearVelocity getTurtleSpeed() {
    return CustomConstants.kTurtleSpeed;
  }

  @Override
  public LinearVelocity getNominalSpeed() {
    return CustomConstants.kNominalSpeed;
  }

  @Override
  public LinearVelocity getTurboSpeed() {
    return CustomConstants.kTurboSpeed;
  }

  @Override
  public double getMaxModularRotationalRate() {
    return GeneratedConstants.kMaxModularRotationalRate;
  }

  @Override
  public RobotConfig getPathPlannerConfig() {
    return CustomConstants.pathPlannerConfig;
  }

  @Override
  public ProfiledPIDController getDriveFacingAnglePIDController() {
    return CustomConstants.driveFacingAngleController;
  }

  @Override
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          GeneratedConstants.FrontLeft.LocationX, GeneratedConstants.FrontLeft.LocationY),
      new Translation2d(
          GeneratedConstants.FrontRight.LocationX, GeneratedConstants.FrontRight.LocationY),
      new Translation2d(
          GeneratedConstants.BackLeft.LocationX, GeneratedConstants.BackLeft.LocationY),
      new Translation2d(
          GeneratedConstants.BackRight.LocationX, GeneratedConstants.BackRight.LocationY)
    };
  }
}
