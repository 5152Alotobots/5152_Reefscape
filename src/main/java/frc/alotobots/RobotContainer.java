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

import static frc.alotobots.OI.*;
import static frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants.NOTE;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.library.subsystems.bling.commands.*;
import frc.alotobots.library.subsystems.bling.io.*;
import frc.alotobots.library.subsystems.swervedrive.*;
import frc.alotobots.library.subsystems.swervedrive.commands.*;
import frc.alotobots.library.subsystems.swervedrive.io.*;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusion;
import frc.alotobots.library.subsystems.vision.localizationfusion.commands.RequestPositionResetViaTags;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.io.*;
import frc.alotobots.library.subsystems.vision.oculus.util.OculusPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.*;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.util.AprilTagPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.ObjectDetectionSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.commands.*;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final OculusSubsystem oculusSubsystem;
  private final AprilTagSubsystem aprilTagSubsystem;
  private final LocalizationFusion localizationFusion;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;
  private final BlingSubsystem blingSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot hardware initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(ModulePosition.FRONT_LEFT.index),
                new ModuleIOTalonFX(ModulePosition.FRONT_RIGHT.index),
                new ModuleIOTalonFX(ModulePosition.BACK_LEFT.index),
                new ModuleIOTalonFX(ModulePosition.BACK_RIGHT.index));
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        configureAutoChooser();

        oculusSubsystem = new OculusSubsystem(new OculusIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[0]),
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[1]));

        // Create pose sources
        OculusPoseSource oculusPoseSource = new OculusPoseSource(oculusSubsystem);
        AprilTagPoseSource aprilTagPoseSource = new AprilTagPoseSource(aprilTagSubsystem);

        localizationFusion =
            new LocalizationFusion(
                swerveDriveSubsystem::addVisionMeasurement,
                oculusPoseSource,
                aprilTagPoseSource,
                autoChooser);

        objectDetectionSubsystem =
            new ObjectDetectionSubsystem(
                swerveDriveSubsystem::getPose,
                new ObjectDetectionIOPhotonVision(ObjectDetectionConstants.CAMERA_CONFIGS[0]));
        blingSubsystem = new BlingSubsystem(new BlingIOReal());
        break;

      case SIM:
        // Simulation hardware initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(ModulePosition.FRONT_LEFT.index),
                new ModuleIOSim(ModulePosition.FRONT_RIGHT.index),
                new ModuleIOSim(ModulePosition.BACK_LEFT.index),
                new ModuleIOSim(ModulePosition.BACK_RIGHT.index));
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        configureAutoChooser();

        oculusSubsystem = new OculusSubsystem(new OculusIOSim());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                new AprilTagIOPhotonVisionSim(
                    AprilTagConstants.CAMERA_CONFIGS[0], swerveDriveSubsystem::getPose),
                new AprilTagIOPhotonVisionSim(
                    AprilTagConstants.CAMERA_CONFIGS[1], swerveDriveSubsystem::getPose));

        // Create pose sources
        oculusPoseSource = new OculusPoseSource(oculusSubsystem);
        aprilTagPoseSource = new AprilTagPoseSource(aprilTagSubsystem);
        localizationFusion =
            new LocalizationFusion(
                swerveDriveSubsystem::addVisionMeasurement,
                oculusPoseSource,
                aprilTagPoseSource,
                autoChooser);

        objectDetectionSubsystem =
            new ObjectDetectionSubsystem(swerveDriveSubsystem::getPose, new ObjectDetectionIO() {});
        blingSubsystem = new BlingSubsystem(new BlingIOSim());
        break;

      default:
        // Replay mode initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        configureAutoChooser();

        oculusSubsystem = new OculusSubsystem(new OculusIO() {});
        aprilTagSubsystem = new AprilTagSubsystem(new AprilTagIO() {}, new AprilTagIO() {});

        // Create pose sources
        oculusPoseSource = new OculusPoseSource(oculusSubsystem);
        aprilTagPoseSource = new AprilTagPoseSource(aprilTagSubsystem);
        localizationFusion =
            new LocalizationFusion(
                swerveDriveSubsystem::addVisionMeasurement,
                oculusPoseSource,
                aprilTagPoseSource,
                autoChooser);

        objectDetectionSubsystem =
            new ObjectDetectionSubsystem(swerveDriveSubsystem::getPose, new ObjectDetectionIO() {});
        blingSubsystem = new BlingSubsystem(new BlingIO() {});
        break;
    }
    configureDefaultCommands();
    configureLogicCommands();
  }

  private void configureDefaultCommands() {
    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem).getCommand());
    blingSubsystem.setDefaultCommand(
        new NoAllianceWaiting(blingSubsystem).andThen(new SetToAllianceColor(blingSubsystem)));
  }

  private void configureLogicCommands() {
    driveFacingBestObjectButton.toggleOnTrue(
        new DriveFacingBestObject(objectDetectionSubsystem, swerveDriveSubsystem, NOTE));
    pathfindToBestObjectButton.onTrue(
        new PathfindToBestObject(
            objectDetectionSubsystem, swerveDriveSubsystem, pathPlannerManager, NOTE));
    testButton.onTrue(new RequestPositionResetViaTags(localizationFusion).withTimeout(1));
  }

  private void configureAutoChooser() {
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Add SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        new WheelRadiusCharacterization(swerveDriveSubsystem));
    autoChooser.addOption(
        "Drive Simple FF Characterization", new FeedforwardCharacterization(swerveDriveSubsystem));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerveDriveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        swerveDriveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
