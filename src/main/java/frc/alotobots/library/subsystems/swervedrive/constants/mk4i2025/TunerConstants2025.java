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
package frc.alotobots.library.subsystems.swervedrive.constants.mk4i2025;

import static edu.wpi.first.units.Units.*;
import static frc.alotobots.library.subsystems.swervedrive.constants.mk4i2023.TunerConstants2023.CustomConstants.ROBOT_MASS;
import static frc.alotobots.library.subsystems.swervedrive.constants.mk4i2023.TunerConstants2023.CustomConstants.moduleTranslations;
import static frc.alotobots.library.subsystems.swervedrive.constants.mk4i2023.TunerConstants2023.GeneratedConstants.kCANBus;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.alotobots.library.subsystems.swervedrive.constants.TunerConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class TunerConstants2025 implements TunerConstants {
  public static class GeneratedConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(100)
            .withKI(0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(2.66)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains =
        new Slot0Configs().withKP(0.18).withKI(0).withKD(0.05).withKS(0.20660).withKV(0.63114);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final SwerveModuleConstants.DriveMotorArrangement kDriveMotorType =
        SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SwerveModuleConstants.SteerMotorArrangement kSteerMotorType =
        SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(70.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("Swerve", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.94);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 0;

    private static final double kDriveGearRatio = 5.87752;
    private static final double kSteerGearRatio = 21.428571;
    private static final Distance kWheelRadius = Inches.of(1.855);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 3;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 10;
    private static final int kFrontLeftSteerMotorId = 11;
    private static final int kFrontLeftEncoderId = 12;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.01416015625);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(11);
    private static final Distance kFrontLeftYPos = Inches.of(9.5);

    // Front Right
    private static final int kFrontRightDriveMotorId = 13;
    private static final int kFrontRightSteerMotorId = 14;
    private static final int kFrontRightEncoderId = 15;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.263427734375);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(11);
    private static final Distance kFrontRightYPos = Inches.of(-9.5);

    // Back Left
    private static final int kBackLeftDriveMotorId = 16;
    private static final int kBackLeftSteerMotorId = 17;
    private static final int kBackLeftEncoderId = 18;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.343017578125);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-11);
    private static final Distance kBackLeftYPos = Inches.of(9.5);

    // Back Right
    private static final int kBackRightDriveMotorId = 19;
    private static final int kBackRightSteerMotorId = 20;
    private static final int kBackRightEncoderId = 21;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.341552734375);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-11);
    private static final Distance kBackRightYPos = Inches.of(-9.5);

    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                kFrontLeftXPos,
                kFrontLeftYPos,
                kInvertLeftSide,
                kFrontLeftSteerMotorInverted,
                kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                kFrontRightXPos,
                kFrontRightYPos,
                kInvertRightSide,
                kFrontRightSteerMotorInverted,
                kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                kBackLeftXPos,
                kBackLeftYPos,
                kInvertLeftSide,
                kBackLeftSteerMotorInverted,
                kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                kBackRightXPos,
                kBackRightYPos,
                kInvertRightSide,
                kBackRightSteerMotorInverted,
                kBackRightEncoderInverted);
  }

  public static class CustomConstants {
    // Custom constants go here
    public static final PIDConstants translationPid = new PIDConstants(2.3, 0, 0.07);
    public static final PIDConstants rotationPid = new PIDConstants(7.8, 0, 0.015);
    public static final double precisionAlignTolerance = .03; // Meters
    public static final double precisionAlignAllowRadius = .5; // Meters
    public static final PathConstraints PATHFINDING_CONSTRAINTS =
        new PathConstraints(5.0, 3.5, Units.degreesToRadians(540), Units.degreesToRadians(460));
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
    public static final double ODOMETRY_FREQUENCY = kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final Distance BUMPER_LENGTH = Distance.ofBaseUnits(.88, Meters);
    public static final Distance BUMPER_WIDTH = Distance.ofBaseUnits(.80, Meters);
    public static final LinearVelocity TURTLE_SPEED = MetersPerSecond.of(1.0);
    public static final LinearVelocity NOMINAL_SPEED = MetersPerSecond.of(3.5);
    public static final LinearVelocity TURBO_SPEED = MetersPerSecond.of(5.2);
    public static final double MAX_MODULAR_ROTATIONAL_RATE = Units.rotationsToRadians(12);
    public static final double ROBOT_MASS_KG = 59.1;
    public static final double ROBOT_MOI = 6.79453;
    public static final double WHEEL_COF = 1.2;

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
                    GeneratedConstants.FrontLeft.LocationX, GeneratedConstants.FrontLeft.LocationY),
                Math.hypot(
                    GeneratedConstants.FrontRight.LocationX,
                    GeneratedConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(
                    GeneratedConstants.BackLeft.LocationX, GeneratedConstants.BackLeft.LocationY),
                Math.hypot(
                    GeneratedConstants.BackRight.LocationX,
                    GeneratedConstants.BackRight.LocationY)));

    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(ROBOT_MASS)
            .withCustomModuleTranslations(moduleTranslations)
            .withGyro(COTS.ofPigeon2())
            .withBumperSize(BUMPER_LENGTH, BUMPER_WIDTH)
            .withSwerveModule(
                COTS.ofMark4i(
                    DCMotor.getFalcon500Foc(1), DCMotor.getFalcon500Foc(1), WHEEL_COF, 2));
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getFrontLeft() {
    return GeneratedConstants.FrontLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getFrontRight() {
    return GeneratedConstants.FrontRight;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getBackLeft() {
    return GeneratedConstants.BackLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getBackRight() {
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
    return CustomConstants.ODOMETRY_FREQUENCY;
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
    return CustomConstants.TURTLE_SPEED;
  }

  @Override
  public LinearVelocity getNominalSpeed() {
    return CustomConstants.NOMINAL_SPEED;
  }

  @Override
  public LinearVelocity getTurboSpeed() {
    return CustomConstants.TURBO_SPEED;
  }

  @Override
  public double getMaxModularRotationalRate() {
    return CustomConstants.MAX_MODULAR_ROTATIONAL_RATE;
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
  public double getPrecisionAlignTolerance() {
    return CustomConstants.precisionAlignTolerance;
  }

  @Override
  public double getPrecisionAlignAllowRadius() {
    return CustomConstants.precisionAlignAllowRadius;
  }

  @Override
  public DriveTrainSimulationConfig getDriveTrainSimulationConfig() {
    return CustomConstants.mapleSimConfig;
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
