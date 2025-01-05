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
package frc.alotobots.library.subsystems.swervedrive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.Constants;
import frc.alotobots.Constants.Mode;
import frc.alotobots.library.subsystems.swervedrive.io.GyroIO;
import frc.alotobots.library.subsystems.swervedrive.io.GyroIOInputsAutoLogged;
import frc.alotobots.library.subsystems.swervedrive.io.ModuleIO;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The SwerveDriveSubsystem class manages the robot's swerve drive system, handling odometry, module
 * control, and autonomous path following capabilities.
 */
public class SwerveDriveSubsystem extends SubsystemBase {

  /** Lock object for synchronizing odometry updates */
  static final Lock odometryLock = new ReentrantLock();

  /** Interface for gyro hardware interactions */
  private final GyroIO gyroIO;

  /** Logged inputs from the gyro */
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  /** Array of swerve drive modules (FL, FR, BL, BR) */
  private final Module[] modules = new Module[4];

  /** System identification routine for characterization */
  private final SysIdRoutine sysId;

  /** Alert for gyro disconnection */
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  /** Generator for swerve drive setpoints */
  private final SwerveSetpointGenerator setpointGenerator;

  /** Previous setpoint state */
  private SwerveSetpoint previousSetpoint;

  /** Kinematics calculator for the swerve drive */
  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.tunerConstants.getModuleTranslations());

  /** Raw rotation from the gyro */
  private Rotation2d rawGyroRotation = new Rotation2d();

  /** Last recorded module positions for delta tracking */
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /** Pose estimator for odometry */
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  /**
   * Constructs a new SwerveDriveSubsystem.
   *
   * @param gyroIO Interface for gyro hardware
   * @param flModuleIO Front left module interface
   * @param frModuleIO Front right module interface
   * @param blModuleIO Back left module interface
   * @param brModuleIO Back right module interface
   */
  public SwerveDriveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;

    // Initialize modules
    modules[0] = new Module(flModuleIO, 0, Constants.tunerConstants.getFrontLeft());
    modules[1] = new Module(frModuleIO, 1, Constants.tunerConstants.getFrontRight());
    modules[2] = new Module(blModuleIO, 2, Constants.tunerConstants.getBackLeft());
    modules[3] = new Module(brModuleIO, 3, Constants.tunerConstants.getBackRight());

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Initialize and start odometry thread
    PhoenixOdometryThread.getInstance().start();

    setpointGenerator =
        new SwerveSetpointGenerator(
            Constants.tunerConstants.getPathPlannerConfig(),
            Constants.tunerConstants.getMaxModularRotationalRate());
    previousSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  /** Periodic update function handling odometry updates and module states. */
  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Desired chassis speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
    SwerveModuleState[] setpointStates = previousSetpoint.moduleStates();

    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, Constants.tunerConstants.getSpeedAt12Volts());

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs drive characterization with specified output.
   *
   * @param output Characterization output value
   */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops all drive modules. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Stops drive and positions modules in X pattern to resist movement. */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = Constants.tunerConstants.getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Creates command for quasistatic system identification.
   *
   * @param direction Test direction
   * @return Command for quasistatic test
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /**
   * Creates command for dynamic system identification.
   *
   * @param direction Test direction
   * @return Command for dynamic test
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Gets current module states.
   *
   * @return Array of module states
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Gets current module positions.
   *
   * @return Array of module positions
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * Gets measured chassis speeds.
   *
   * @return Current chassis speeds
   */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets wheel positions for radius characterization.
   *
   * @return Array of wheel positions in radians
   */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Gets average module velocity for feedforward characterization.
   *
   * @return Average velocity in rotations/sec (Phoenix native units)
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /**
   * Gets current odometry pose.
   *
   * @return Current robot pose
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets current odometry rotation.
   *
   * @return Current robot rotation
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Resets odometry to specified pose.
   *
   * @param pose New robot pose
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds vision measurement for pose estimation.
   *
   * @param visionRobotPoseMeters Vision-measured robot pose
   * @param timestampSeconds Timestamp of measurement
   * @param visionMeasurementStdDevs Standard deviations of vision measurements
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /**
   * Gets maximum linear speed capability.
   *
   * @return Maximum speed in meters per second
   */
  public double getMaxLinearSpeedMetersPerSec() {
    return Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
  }

  /**
   * Gets maximum angular speed capability.
   *
   * @return Maximum angular speed in radians per second
   */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / Constants.tunerConstants.getDriveBaseRadius();
  }
}
