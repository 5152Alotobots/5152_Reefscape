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
package frc.alotobots;

import static frc.alotobots.OI.*;
import static frc.alotobots.reefscape.subsystems.wrist.constants.WristIOTalonFXConstants.WristPositions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.library.subsystems.swervedrive.*;
import frc.alotobots.library.subsystems.swervedrive.commands.*;
import frc.alotobots.library.subsystems.swervedrive.io.*;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusion;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.io.*;
import frc.alotobots.library.subsystems.vision.oculus.util.OculusPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.*;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.util.AprilTagPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.ObjectDetectionSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants.ObjectDetectionConstants;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.io.*;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.DefaultCoralIntakeOpenLoop;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.DefaultCoralIntakeOpenLoopWOLimits;
import frc.alotobots.reefscape.subsystems.coralIntake.io.CoralIntakeIOVortexReal;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.DefaultElevatorRunAtVelocity;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIO;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOTalonFXReal;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOTalonFXSim;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.commands.DefaultWristOpenLoop;
import frc.alotobots.reefscape.subsystems.wrist.commands.WristRunToAngle;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOTalonFXReal;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOTalonFXSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final OculusSubsystem oculusSubsystem;
  private final AprilTagSubsystem aprilTagSubsystem;
  private final LocalizationFusion localizationFusion;
  private final OculusPoseSource oculusPoseSource;
  private final AprilTagPoseSource aprilTagPoseSource;
  private final ObjectDetectionSubsystem objectDetectionSubsystem;
  private final WristSubsystem wristSubsystem;
  private final CoralIntakeSubsystem coralIntakeSubsystem;
  // private final BlingSubsystem blingSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private LoggedDashboardChooser<Command> autoChooser;
  private SwerveDriveSimulation driveSimulation;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeIOVortexReal());
        wristSubsystem = new WristSubsystem(new WristIOTalonFXReal());
        // Real robot hardware initialization

        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(ModulePosition.FRONT_LEFT.index),
                new ModuleIOTalonFXReal(ModulePosition.FRONT_RIGHT.index),
                new ModuleIOTalonFXReal(ModulePosition.BACK_LEFT.index),
                new ModuleIOTalonFXReal(ModulePosition.BACK_RIGHT.index));
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFXReal());
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        configureAutoChooser();

        oculusSubsystem = new OculusSubsystem(new OculusIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[0]),
                new AprilTagIOPhotonVision(AprilTagConstants.CAMERA_CONFIGS[1]));

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
            new ObjectDetectionSubsystem(
                swerveDriveSubsystem::getPose,
                new ObjectDetectionIOPhotonVision(ObjectDetectionConstants.CAMERA_CONFIGS[0]));
        // blingSubsystem = new BlingSubsystem(new BlingIOReal());
        break;

      case SIM:
        wristSubsystem = new WristSubsystem(new WristIOTalonFXSim());
        coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeIOVortexReal());
        Pose2d simStartPose = new Pose2d(3, 3, new Rotation2d(0));
        driveSimulation =
            new SwerveDriveSimulation(
                Constants.tunerConstants.getDriveTrainSimulationConfig(), simStartPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        // Simulation hardware initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIOTalonFXSim(
                    ModulePosition.FRONT_LEFT.index,
                    driveSimulation.getModules()[ModulePosition.FRONT_LEFT.index]),
                new ModuleIOTalonFXSim(
                    ModulePosition.FRONT_RIGHT.index,
                    driveSimulation.getModules()[ModulePosition.FRONT_RIGHT.index]),
                new ModuleIOTalonFXSim(
                    ModulePosition.BACK_LEFT.index,
                    driveSimulation.getModules()[ModulePosition.BACK_LEFT.index]),
                new ModuleIOTalonFXSim(
                    ModulePosition.BACK_RIGHT.index,
                    driveSimulation.getModules()[ModulePosition.BACK_RIGHT.index]));
        swerveDriveSubsystem.setPose(simStartPose);
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFXSim());
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        configureAutoChooser();


        oculusSubsystem = new OculusSubsystem(new OculusIOSim(driveSimulation));
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
        // blingSubsystem = new BlingSubsystem(new BlingIOSim());
        break;

      default:
        wristSubsystem = new WristSubsystem(new WristIOTalonFXSim());
        coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeIOVortexReal());
        // Replay mode initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
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
        // blingSubsystem = new BlingSubsystem(new BlingIO() {});
        break;
    }
    configureDefaultCommands();
    configureLogicCommands();
  }

  private void configureDefaultCommands() {
    wristSubsystem.setDefaultCommand(
        new DefaultWristOpenLoop(wristSubsystem, () -> OI.getWristAxis()));
    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem).getCommand());
    elevatorSubsystem.setDefaultCommand(
        new DefaultElevatorRunAtVelocity(elevatorSubsystem, () -> -getElevatorAxis()));
    // blingSubsystem.setDefaultCommand(
    //    new NoAllianceWaiting(blingSubsystem).andThen(new SetToAllianceColor(blingSubsystem)));
  }

  private void configureLogicCommands() {
    // Elevator
    elevatorStowButton.onTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.STOWED));
    elevatorL2Button.onTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.L2_PLACE));
    elevatorL3Button.onTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.L3_PLACE));
    elevatorL4Button.onTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.L4_PLACE));
    wristL4coralButton.whileTrue(new WristRunToAngle(wristSubsystem, L4_CORAL_POSITION));
    wristL2and3coralButton.whileTrue(new WristRunToAngle(wristSubsystem, L2AND3_CORAL_POSITION));
    wristGroundButton.whileTrue(new WristRunToAngle(wristSubsystem, GROUND_INTAKE));
    intakeButton.whileTrue(new DefaultCoralIntakeOpenLoop(coralIntakeSubsystem, () -> 1.0));
    intakeButtonNoLimits.whileTrue(new DefaultCoralIntakeOpenLoopWOLimits(coralIntakeSubsystem, () -> 1.0));
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

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
