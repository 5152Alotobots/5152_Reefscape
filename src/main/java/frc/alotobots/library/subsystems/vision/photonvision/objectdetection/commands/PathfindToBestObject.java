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
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.commands;

import static frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants.GAME_ELEMENTS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.Constants;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DefaultDrive;
import frc.alotobots.library.subsystems.swervedrive.commands.DriveFacingPose;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.ObjectDetectionSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;

/**
 * A command that automatically navigates the robot to the best detected game object. The robot will
 * pathfind to an offset position near the highest priority detected game element, accounting for
 * robot bumper dimensions when calculating the final position.
 */
public class PathfindToBestObject extends Command {

  /** Subsystem for detecting game objects */
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  /** Subsystem for controlling robot movement */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** The pathplanner instance */
  private final PathPlannerManager pathPlannerManager;

  /** Array of game elements to target, in priority order */
  private final GameElement[] targetGameElementNames;

  /** Command for driving while facing a specific pose */
  private final DriveFacingPose driveFacingPose;

  /** Command for standard manual driving */
  private final DefaultDrive defaultDrive;

  /** Timer for tracking execution duration */
  Timer overrideTimer = new Timer();

  /**
   * Creates a new PathfindToBestObject command.
   *
   * @param objectDetectionSubsystem Subsystem used for detecting game objects
   * @param swerveDriveSubsystem Subsystem used for robot movement
   * @param targetGameElementNames Array of game elements to target, in priority order
   */
  public PathfindToBestObject(
      ObjectDetectionSubsystem objectDetectionSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem,
      PathPlannerManager pathPlannerManager,
      GameElement... targetGameElementNames) {
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.pathPlannerManager = pathPlannerManager;
    this.targetGameElementNames = targetGameElementNames;
    this.driveFacingPose = new DriveFacingPose(swerveDriveSubsystem);
    this.defaultDrive = new DefaultDrive(swerveDriveSubsystem);

    addRequirements(swerveDriveSubsystem, objectDetectionSubsystem);
  }

  /**
   * Executes the command logic. Gets the latest object detections and generates path: - Finds
   * highest priority detected object - Calculates offset position accounting for robot bumper
   * dimensions - Generates and executes pathfinding command to target position
   */
  @Override
  public void execute() {
    var detectedObjects = objectDetectionSubsystem.getStableDetectedObjects();

    // Find first matching object based on priority order
    var matchingObject = java.util.Optional.<ObjectDetectionIO.DetectedObjectFieldRelative>empty();
    for (GameElement element : targetGameElementNames) {
      matchingObject =
          detectedObjects.stream()
              .filter(obj -> GAME_ELEMENTS[obj.classId()].equals(element))
              .findFirst();
      if (matchingObject.isPresent()) {
        break;
      }
    }

    if (matchingObject.isPresent()) {
      Pose2d robotPose = swerveDriveSubsystem.getPose();
      Pose2d objectPose = matchingObject.get().pose().toPose2d();

      // Calculate direction from robot to object
      Translation2d toObject = objectPose.getTranslation().minus(robotPose.getTranslation());
      double angle = Math.atan2(toObject.getY(), toObject.getX());

      // Determine which bumper dimension to use based on approach angle
      Distance offset =
          Math.abs(Math.cos(angle)) > Math.abs(Math.sin(angle))
              ? Constants.tunerConstants.getBumperLength()
              : Constants.tunerConstants.getBumperWidth();

      // Calculate target pose offset from object
      double offsetMeters = offset.in(Units.Meters);
      Translation2d offsetTranslation =
          new Translation2d(-Math.cos(angle) * offsetMeters, -Math.sin(angle) * offsetMeters);

      // Create target pose with offset
      Pose2d targetPose =
          new Pose2d(objectPose.getTranslation().plus(offsetTranslation), objectPose.getRotation());

      Command command =
          pathPlannerManager.getPathFinderCommand(
              targetPose, LinearVelocity.ofBaseUnits(0, Units.MetersPerSecond));
      command.schedule();
    }
  }

  /** Duration in seconds before command ends */
  private static final double OVERRIDE_TIMEOUT_SECONDS = 0.1;

  /**
   * Determines if the command should end.
   *
   * @return true if timeout has elapsed
   */
  @Override
  public boolean isFinished() {
    return overrideTimer.hasElapsed(OVERRIDE_TIMEOUT_SECONDS);
  }
}
