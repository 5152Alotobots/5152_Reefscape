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
package frc.alotobots;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class that handles robot lifecycle and mode transitions. This class extends
 * LoggedRobot to integrate with AdvantageKit logging framework. It manages the robot's different
 * operating modes (autonomous, teleop, test) and handles the scheduling of commands.
 */
public class Robot extends LoggedRobot {
  /** The command to run during autonomous mode. */
  private Command autonomousCommand;

  /** The main robot container instance that holds all subsystems and commands. */
  private RobotContainer robotContainer;

  /**
   * Constructor for the Robot class. Initializes AdvantageKit logging and sets up the robot
   * container. Different logging configurations are used based on whether the robot is running in
   * real, simulation, or replay mode.
   */
  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called periodically during all modes. It runs the command scheduler with high
   * priority to ensure consistent timing.
   */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  /** Called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** Called periodically when the robot is disabled. */
  @Override
  public void disabledPeriodic() {}

  /**
   * Called once when autonomous mode is enabled. Schedules the autonomous command selected in
   * RobotContainer.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** Called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {}

  /**
   * Called once when teleop mode is enabled. Cancels the autonomous command if it's still running.
   */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** Called periodically during teleop mode. */
  @Override
  public void teleopPeriodic() {}

  /**
   * Called once when test mode is enabled. Cancels all running commands when entering test mode.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when simulation mode is first enabled. */
  @Override
  public void simulationInit() {}

  /** Called periodically during simulation mode. */
  @Override
  public void simulationPeriodic() {}
}
