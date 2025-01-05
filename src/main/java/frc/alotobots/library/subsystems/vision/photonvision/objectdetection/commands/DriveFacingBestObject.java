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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DefaultDrive;
import frc.alotobots.library.subsystems.swervedrive.commands.DriveFacingPose;
import frc.alotobots.library.subsystems.swervedrive.util.DriveCalculator;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.ObjectDetectionSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.ObjectDetectionIO;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;

/**
 * A command that automatically rotates the robot to face detected game objects while allowing
 * manual translation control. The robot will face the highest priority detected game element while
 * driving. If no objects are detected, or if manual rotation override is active, falls back to
 * standard manual control.
 */
public class DriveFacingBestObject extends Command {

  /** Subsystem for detecting game objects */
  private final ObjectDetectionSubsystem objectDetectionSubsystem;

  /** Subsystem for controlling robot movement */
  private final SwerveDriveSubsystem swerveDriveSubsystem;

  /** Array of game elements to target, in priority order */
  private final GameElement[] targetGameElementNames;

  /** Command for driving while facing a specific pose */
  private final DriveFacingPose driveFacingPose;

  /** Command for standard manual driving */
  private final DefaultDrive defaultDrive;

  /** Timer for tracking manual rotation override duration */
  Timer overrideTimer = new Timer();

  /**
   * Creates a new DriveFacingBestObject command.
   *
   * @param objectDetectionSubsystem Subsystem used for detecting game objects
   * @param swerveDriveSubsystem Subsystem used for robot movement
   * @param targetGameElementNames Array of game elements to target, in priority order
   */
  public DriveFacingBestObject(
      ObjectDetectionSubsystem objectDetectionSubsystem,
      SwerveDriveSubsystem swerveDriveSubsystem,
      GameElement... targetGameElementNames) {
    this.objectDetectionSubsystem = objectDetectionSubsystem;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.targetGameElementNames = targetGameElementNames;
    this.driveFacingPose = new DriveFacingPose(swerveDriveSubsystem);
    this.defaultDrive = new DefaultDrive(swerveDriveSubsystem);

    addRequirements(swerveDriveSubsystem, objectDetectionSubsystem);
  }

  /**
   * Executes the command logic. Gets the latest object detections and controls robot movement: - If
   * objects are detected, rotates to face highest priority object while allowing manual translation
   * - If no objects detected, allows full manual control - If manual rotation override active,
   * starts timeout timer
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
      Pose2d pose = matchingObject.get().pose().toPose2d();
      driveFacingPose.applyRequest(() -> pose); // Drive facing the pose
    } else {
      defaultDrive.applyRequest(); // Drive normal
    }

    // Rotation override timeout
    if (matchingObject.isPresent() && DriveCalculator.getDriverRotation() != 0) {
      overrideTimer.start();
    } else {
      overrideTimer.reset();
    }
  }

  /** Duration in seconds that manual rotation override remains active */
  private static final double OVERRIDE_TIMEOUT_SECONDS = 0.1;

  /**
   * Determines if the command should end.
   *
   * @return true if manual rotation override timeout has elapsed
   */
  @Override
  public boolean isFinished() {
    return overrideTimer.hasElapsed(OVERRIDE_TIMEOUT_SECONDS);
  }
}
