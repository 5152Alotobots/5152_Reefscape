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
package frc.alotobots.library.subsystems.swervedrive.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.alotobots.Constants;
import frc.alotobots.OI;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import java.util.Optional;

public class DriveCalculator {
  /** Calculates the linear velocity vector based on joystick inputs and drive mode. */
  private static Translation2d getLinearVelocityFromJoysticks(
      double x, double y, SwerveDriveSubsystem swerveDriveSubsystem) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), OI.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(-y, -x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Calculate speed scaling
    double speedScale = calculateSpeedScale();

    // Apply speed scaling to magnitude
    linearMagnitude *= speedScale;

    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /** Calculates the rotational velocity based on joystick input and drive mode. */
  private static double getRotationalVelocityFromJoystick(double rotation) {
    // Apply deadband and square inputs for more precise control
    rotation = MathUtil.applyDeadband(-rotation, OI.DEADBAND);
    rotation = Math.copySign(rotation * rotation, rotation);

    // Calculate and apply speed scaling
    double speedScale = calculateSpeedScale();

    return rotation * speedScale;
  }

  /** Calculates the speed scaling factor based on trigger inputs */
  private static double calculateSpeedScale() {
    double speedScale =
        Constants.tunerConstants.getNominalSpeed().in(MetersPerSecond)
            / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);

    double turtleTrigger = OI.getTurtleSpeedTrigger();
    double turboTrigger = OI.getTurboSpeedTrigger();

    if (turtleTrigger > OI.DEADBAND) {
      speedScale =
          Constants.tunerConstants.getTurtleSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    } else if (turboTrigger > OI.DEADBAND) {
      speedScale =
          Constants.tunerConstants.getTurboSpeed().in(MetersPerSecond)
              / Constants.tunerConstants.getSpeedAt12Volts().in(MetersPerSecond);
    }

    return speedScale;
  }

  /** Gets the desired linear velocity vector from the driver's left joystick input. */
  public static Translation2d getDriverLinearVelocity(SwerveDriveSubsystem swerveDriveSubsystem) {
    return getLinearVelocityFromJoysticks(
        OI.getTranslateForwardAxis(), OI.getTranslateStrafeAxis(), swerveDriveSubsystem);
  }

  /** Gets the desired rotational velocity from the driver's right joystick input. */
  public static double getDriverRotation() {
    return getRotationalVelocityFromJoystick(OI.getRotationAxis());
  }

  /**
   * Gets the chassis speeds from driver input, applying field-relative conversions and speed
   * scaling.
   */
  public static ChassisSpeeds getChassisSpeeds(SwerveDriveSubsystem swerveDriveSubsystem) {
    Translation2d linearVelocity = getDriverLinearVelocity(swerveDriveSubsystem);
    double omega = getDriverRotation();

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * swerveDriveSubsystem.getMaxLinearSpeedMetersPerSec(),
            omega * swerveDriveSubsystem.getMaxAngularSpeedRadPerSec());

    // Convert to robot relative speeds based on alliance
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    speeds.toRobotRelativeSpeeds(
        isFlipped
            ? swerveDriveSubsystem.getRotation().plus(new Rotation2d(Math.PI))
            : swerveDriveSubsystem.getRotation());

    return speeds;
  }

  /**
   * Gets the chassis speeds from driver input, with optional overrides for any axis. Any provided
   * value will override the joystick input for that axis.
   *
   * @param swerveDriveSubsystem The swerve drive subsystem
   * @param xOverride Optional override for the X (forward/backward) velocity
   * @param yOverride Optional override for the Y (left/right) velocity
   * @param rotationOverride Optional override for the rotational velocity
   * @return ChassisSpeeds representing the desired robot motion
   */
  public static ChassisSpeeds getChassisSpeeds(
      SwerveDriveSubsystem swerveDriveSubsystem,
      Optional<Double> xOverride,
      Optional<Double> yOverride,
      Optional<Double> rotationOverride) {

    // Get default chassis speeds from joystick
    ChassisSpeeds defaultSpeeds = getChassisSpeeds(swerveDriveSubsystem);

    // Create new chassis speeds with overrides where provided
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xOverride.orElse(defaultSpeeds.vxMetersPerSecond),
            yOverride.orElse(defaultSpeeds.vyMetersPerSecond),
            rotationOverride.orElse(defaultSpeeds.omegaRadiansPerSecond));

    return speeds;
  }

  /**
   * Convenience method for overriding a single axis while using default values for others
   *
   * @param swerveDriveSubsystem The swerve drive subsystem
   * @param xOverride Optional override for the X (forward/backward) velocity
   * @return ChassisSpeeds with the X override applied
   */
  public static ChassisSpeeds getChassisSpeedsWithXOverride(
      SwerveDriveSubsystem swerveDriveSubsystem, double xOverride) {
    return getChassisSpeeds(
        swerveDriveSubsystem, Optional.of(xOverride), Optional.empty(), Optional.empty());
  }

  /**
   * Convenience method for overriding a single axis while using default values for others
   *
   * @param swerveDriveSubsystem The swerve drive subsystem
   * @param yOverride Optional override for the Y (left/right) velocity
   * @return ChassisSpeeds with the Y override applied
   */
  public static ChassisSpeeds getChassisSpeedsWithYOverride(
      SwerveDriveSubsystem swerveDriveSubsystem, double yOverride) {
    return getChassisSpeeds(
        swerveDriveSubsystem, Optional.empty(), Optional.of(yOverride), Optional.empty());
  }

  /**
   * Convenience method for overriding a single axis while using default values for others
   *
   * @param swerveDriveSubsystem The swerve drive subsystem
   * @param rotationOverride Optional override for the rotational velocity
   * @return ChassisSpeeds with the rotation override applied
   */
  public static ChassisSpeeds getChassisSpeedsWithRotationOverride(
      SwerveDriveSubsystem swerveDriveSubsystem, double rotationOverride) {
    return getChassisSpeeds(
        swerveDriveSubsystem, Optional.empty(), Optional.empty(), Optional.of(rotationOverride));
  }
}
