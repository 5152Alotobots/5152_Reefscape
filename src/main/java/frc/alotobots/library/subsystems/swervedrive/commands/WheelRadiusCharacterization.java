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
package frc.alotobots.library.subsystems.swervedrive.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;

/**
 * Command that measures the robot's wheel radius by spinning in a circle. This command will run
 * until interrupted, then calculate and print the results.
 */
public class WheelRadiusCharacterization extends Command {
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.5;
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 2.0;
  private static final double DRIVE_BASE_RADIUS = 0.4;
  private static final double ORIENTATION_DELAY = 1.0;

  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
  private final Timer timer = new Timer();

  private double[] startPositions = new double[4];
  private Rotation2d lastAngle = new Rotation2d();
  private double gyroDelta = 0.0;
  private boolean measurementStarted = false;

  /**
   * Creates a new WheelRadiusCharacterization command.
   *
   * @param swerveDriveSubsystem The subsystem to characterize
   */
  public WheelRadiusCharacterization(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    limiter.reset(0.0);
    measurementStarted = false;
    gyroDelta = 0.0;
    timer.restart();
  }

  @Override
  public void execute() {
    if (timer.get() < ORIENTATION_DELAY) {
      // Allow modules to orient
      swerveDriveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
    } else {
      if (!measurementStarted) {
        measurementStarted = true;
        startPositions = swerveDriveSubsystem.getWheelRadiusCharacterizationPositions();
        lastAngle = swerveDriveSubsystem.getRotation();
      }

      // Turn in place, accelerating up to full speed
      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
      swerveDriveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));

      // Update gyro delta
      var rotation = swerveDriveSubsystem.getRotation();
      gyroDelta += Math.abs(rotation.minus(lastAngle).getRadians());
      lastAngle = rotation;
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));

    // Calculate and print results
    if (measurementStarted) {
      double[] endPositions = swerveDriveSubsystem.getWheelRadiusCharacterizationPositions();
      double wheelDelta = 0.0;
      for (int i = 0; i < 4; i++) {
        wheelDelta += Math.abs(endPositions[i] - startPositions[i]) / 4.0;
      }
      double wheelRadius = (gyroDelta * DRIVE_BASE_RADIUS) / wheelDelta;

      NumberFormat formatter = new DecimalFormat("#0.000");
      System.out.println("********** Wheel Radius Characterization Results **********");
      System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
      System.out.println("\tGyro Delta: " + formatter.format(gyroDelta) + " radians");
      System.out.println(
          "\tWheel Radius: "
              + formatter.format(wheelRadius)
              + " meters, "
              + formatter.format(Units.metersToInches(wheelRadius))
              + " inches");
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
