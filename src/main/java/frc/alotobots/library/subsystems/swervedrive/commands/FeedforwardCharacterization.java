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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

/**
 * Command that measures the velocity feedforward constants for the drive motors. This command
 * should only be used in voltage control mode.
 */
public class FeedforwardCharacterization extends Command {
  private static final double FF_START_DELAY = 2.0;
  private static final double FF_RAMP_RATE = 0.5;

  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final Timer timer = new Timer();
  private final List<Double> velocitySamples = new LinkedList<>();
  private final List<Double> voltageSamples = new LinkedList<>();

  private double startTime;
  private boolean characterizationStarted = false;

  /**
   * Creates a new FeedforwardCharacterization command.
   *
   * @param swerveDriveSubsystem The subsystem to characterize
   */
  public FeedforwardCharacterization(SwerveDriveSubsystem swerveDriveSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    velocitySamples.clear();
    voltageSamples.clear();
    characterizationStarted = false;
    startTime = Timer.getTimestamp();
    timer.restart();
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();

    if (currentTime - startTime < FF_START_DELAY) {
      // Allow modules to orient
      swerveDriveSubsystem.runCharacterization(0.0);
    } else {
      if (!characterizationStarted) {
        characterizationStarted = true;
        timer.restart();
      }

      // Accelerate and gather data
      double voltage = timer.get() * FF_RAMP_RATE;
      swerveDriveSubsystem.runCharacterization(voltage);
      velocitySamples.add(swerveDriveSubsystem.getFFCharacterizationVelocity());
      voltageSamples.add(voltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.runCharacterization(0.0);

    // Calculate and print results
    int n = velocitySamples.size();
    if (n > 0) {
      double sumX = 0.0;
      double sumY = 0.0;
      double sumXY = 0.0;
      double sumX2 = 0.0;
      for (int i = 0; i < n; i++) {
        sumX += velocitySamples.get(i);
        sumY += voltageSamples.get(i);
        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
      }
      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

      NumberFormat formatter = new DecimalFormat("#0.00000");
      System.out.println("********** Drive FF Characterization Results **********");
      System.out.println("\tkS: " + formatter.format(kS));
      System.out.println("\tkV: " + formatter.format(kV));
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}
