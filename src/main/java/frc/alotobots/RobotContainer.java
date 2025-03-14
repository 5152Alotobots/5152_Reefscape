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

import static edu.wpi.first.units.Units.Seconds;
import static frc.alotobots.OI.*;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.BLING_NOTIFICATION_TIME;
import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Setpoints.OpenLoop.EJECT_PERCENTAGE;
import static frc.alotobots.reefscape.subsystems.coralIntake.constants.CoralIntakeConstants.Setpoints.OpenLoop.INTAKE_PERCENTAGE;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.alotobots.library.subsystems.bling.BlingSubsystem;
import frc.alotobots.library.subsystems.bling.commands.*;
import frc.alotobots.library.subsystems.bling.io.BlingIO;
import frc.alotobots.library.subsystems.bling.io.BlingIOReal;
import frc.alotobots.library.subsystems.bling.io.BlingIOSim;
import frc.alotobots.library.subsystems.bling.util.BlingUtil;
import frc.alotobots.library.subsystems.swervedrive.*;
import frc.alotobots.library.subsystems.swervedrive.commands.*;
import frc.alotobots.library.subsystems.swervedrive.io.*;
import frc.alotobots.library.subsystems.swervedrive.util.DriveCalculator;
import frc.alotobots.library.subsystems.swervedrive.util.PathPlannerManager;
import frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusion;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.io.*;
import frc.alotobots.library.subsystems.vision.oculus.util.OculusPoseSource;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.constants.AprilTagConstants;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.io.*;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.util.AprilTagPoseSource;
import frc.alotobots.reefscape.commands.groups.Climb;
import frc.alotobots.reefscape.commands.groups.UnClimb;
import frc.alotobots.reefscape.commands.states.algae.StateAlgaeRemoveL2;
import frc.alotobots.reefscape.commands.states.algae.StateAlgaeRemoveL3;
import frc.alotobots.reefscape.commands.states.coral.*;
import frc.alotobots.reefscape.subsystems.autocycle.AutoCycleSubsystem;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToCoralStation;
import frc.alotobots.reefscape.subsystems.autocycle.commands.PathfindToReef;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;
import frc.alotobots.reefscape.subsystems.climber.commands.ClimberDisableServos;
import frc.alotobots.reefscape.subsystems.climber.io.ClimberIORevServoReal;
import frc.alotobots.reefscape.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeEject;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeEjectManual;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeEjectThrough;
import frc.alotobots.reefscape.subsystems.coralIntake.commands.CoralIntakeIntakeManual;
import frc.alotobots.reefscape.subsystems.coralIntake.io.CoralIntakeIO;
import frc.alotobots.reefscape.subsystems.coralIntake.io.CoralIntakeIOTalonFXReal;
import frc.alotobots.reefscape.subsystems.elevator.ElevatorSubsystem;
import frc.alotobots.reefscape.subsystems.elevator.commands.DefaultElevatorRunAtVelocity;
import frc.alotobots.reefscape.subsystems.elevator.commands.ElevatorRunToHeight;
import frc.alotobots.reefscape.subsystems.elevator.constants.ElevatorConstants;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIO;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOTalonFXReal;
import frc.alotobots.reefscape.subsystems.elevator.io.ElevatorIOTalonFXSim;
import frc.alotobots.reefscape.subsystems.wrist.WristSubsystem;
import frc.alotobots.reefscape.subsystems.wrist.commands.DefaultWristRunAtVelocity;
import frc.alotobots.reefscape.subsystems.wrist.commands.WristRunToAngle;
import frc.alotobots.reefscape.subsystems.wrist.constants.WristConstants;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOTalonFXReal;
import frc.alotobots.reefscape.subsystems.wrist.io.WristIOTalonFXSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final CoralIntakeSubsystem coralIntakeSubsystem;
  private final OculusSubsystem oculusSubsystem;
  private final AprilTagSubsystem aprilTagSubsystem;
  private final LocalizationFusion localizationFusion;
  private final OculusPoseSource oculusPoseSource;
  private final AprilTagPoseSource aprilTagPoseSource;
  private final AutoCycleSubsystem autoCycleSubsystem;
  private final BlingSubsystem blingSubsystem;
  private final PathPlannerManager pathPlannerManager;
  private final AutoNamedCommands autoNamedCommands;
  private LoggedDashboardChooser<Command> autoChooser;
  private SwerveDriveSimulation driveSimulation;

  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot hardware initialization

        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(ModulePosition.FRONT_LEFT.index),
                new ModuleIOTalonFXReal(ModulePosition.FRONT_RIGHT.index),
                new ModuleIOTalonFXReal(ModulePosition.BACK_LEFT.index),
                new ModuleIOTalonFXReal(ModulePosition.BACK_RIGHT.index));
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOTalonFXReal());
        coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeIOTalonFXReal());
        wristSubsystem = new WristSubsystem(new WristIOTalonFXReal());
        climberSubsystem = new ClimberSubsystem(new ClimberIORevServoReal());
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        autoNamedCommands =
            new AutoNamedCommands(elevatorSubsystem, wristSubsystem, coralIntakeSubsystem);
        configureAutoChooser();
        autoCycleSubsystem =
            new AutoCycleSubsystem(
                pathPlannerManager,
                swerveDriveSubsystem,
                () -> DriveCalculator.getChassisSpeeds(swerveDriveSubsystem));
        oculusSubsystem = new OculusSubsystem(new OculusIOReal());
        aprilTagSubsystem =
            new AprilTagSubsystem(
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[0], swerveDriveSubsystem::getRotation),
                new AprilTagIOPhotonVision(
                    AprilTagConstants.CAMERA_CONFIGS[1], swerveDriveSubsystem::getRotation));

        // Create pose sources
        oculusPoseSource = new OculusPoseSource(oculusSubsystem);
        aprilTagPoseSource = new AprilTagPoseSource(aprilTagSubsystem);

        localizationFusion =
            new LocalizationFusion(
                swerveDriveSubsystem::addVisionMeasurement,
                oculusPoseSource,
                aprilTagPoseSource,
                swerveDriveSubsystem,
                autoChooser);

        blingSubsystem = new BlingSubsystem(new BlingIOReal());
        break;

      case SIM:
        coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeIO() {});
        Pose2d simStartPose = new Pose2d(3, 3, new Rotation2d(0));
        driveSimulation =
            new SwerveDriveSimulation(
                Constants.tunerConstants.getDriveTrainSimulationConfig(), simStartPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        climberSubsystem = new ClimberSubsystem(new ClimberIORevServoReal());
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
        wristSubsystem = new WristSubsystem(new WristIOTalonFXSim());
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        autoNamedCommands =
            new AutoNamedCommands(elevatorSubsystem, wristSubsystem, coralIntakeSubsystem);
        configureAutoChooser();
        autoCycleSubsystem =
            new AutoCycleSubsystem(
                pathPlannerManager,
                swerveDriveSubsystem,
                () -> DriveCalculator.getChassisSpeeds(swerveDriveSubsystem));

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
                swerveDriveSubsystem,
                autoChooser);

        blingSubsystem = new BlingSubsystem(new BlingIOSim());
        break;

      default:
        coralIntakeSubsystem = new CoralIntakeSubsystem(new CoralIntakeIOTalonFXReal());
        climberSubsystem = new ClimberSubsystem(new ClimberIORevServoReal());
        // Replay mode initialization
        swerveDriveSubsystem =
            new SwerveDriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
        wristSubsystem = new WristSubsystem(new WristIOTalonFXSim());
        pathPlannerManager = new PathPlannerManager(swerveDriveSubsystem);
        autoNamedCommands =
            new AutoNamedCommands(elevatorSubsystem, wristSubsystem, coralIntakeSubsystem);
        configureAutoChooser();
        autoCycleSubsystem =
            new AutoCycleSubsystem(
                pathPlannerManager,
                swerveDriveSubsystem,
                () -> DriveCalculator.getChassisSpeeds(swerveDriveSubsystem));

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
                swerveDriveSubsystem,
                autoChooser);

        blingSubsystem = new BlingSubsystem(new BlingIO() {});
        break;
    }
    configureDefaultCommands();
    configureLogicCommands();
  }

  /** Commands that run when nothing else is */
  private void configureDefaultCommands() {
    swerveDriveSubsystem.setDefaultCommand(new DefaultDrive(swerveDriveSubsystem).getCommand());
    elevatorSubsystem.setDefaultCommand(
        new DefaultElevatorRunAtVelocity(elevatorSubsystem, () -> -getElevatorAxis()));
    wristSubsystem.setDefaultCommand(
        new DefaultWristRunAtVelocity(wristSubsystem, OI::getWristAxis));
    blingSubsystem.setDefaultCommand(
        new NoAllianceWaiting(blingSubsystem).andThen(new SetToAllianceColor(blingSubsystem)));
    climberSubsystem.setDefaultCommand(new ClimberDisableServos(climberSubsystem));
  }

  /** Contains button based commands */
  private void configureLogicCommands() {
    // TEMPORARY!!
    resetGyroButton.onTrue(
        new InstantCommand(() -> swerveDriveSubsystem.setPose(new Pose2d(0, 0, Rotation2d.kZero))));
    // Bling
    BlingUtil.scheduleAtMatchTime(
        new BlingEndgameCountdown(blingSubsystem)
            .withTimeout(20)
            .andThen(new BlingTimeToClimb(blingSubsystem).withTimeout(BLING_NOTIFICATION_TIME)),
        Seconds.of(30));
    new Trigger(coralIntakeSubsystem::isIntakeOccupied)
        .onTrue(new BlingCoralHasPiece(blingSubsystem).withTimeout(BLING_NOTIFICATION_TIME));

    stateCoralCoralStationButton.toggleOnTrue(
        new StateCoralCoralStation(
            elevatorSubsystem, wristSubsystem, coralIntakeSubsystem, blingSubsystem));
    stateCoralL1Button.toggleOnTrue(
        new StateCoralL1(
            elevatorSubsystem,
            wristSubsystem,
            coralIntakeSubsystem,
            blingSubsystem,
            coralIntakeReleaseButton));
    stateCoralL2Button.toggleOnTrue(
        new StateCoralL2(
            elevatorSubsystem,
            wristSubsystem,
            coralIntakeSubsystem,
            blingSubsystem,
            coralIntakeReleaseButton));
    stateCoralL3Button.toggleOnTrue(
        new StateCoralL3(
            elevatorSubsystem,
            wristSubsystem,
            coralIntakeSubsystem,
            blingSubsystem,
            coralIntakeReleaseButton));
    stateCoralL4Button.toggleOnTrue(
        new StateCoralL4(
            elevatorSubsystem,
            wristSubsystem,
            coralIntakeSubsystem,
            blingSubsystem,
            coralIntakeReleaseButton));
    stateCoralStowedButton.toggleOnTrue(new StateCoralStowed(elevatorSubsystem, wristSubsystem));
    stateCoralGroundButton.toggleOnTrue(
        new StateCoralGround(
            elevatorSubsystem, wristSubsystem, coralIntakeSubsystem, blingSubsystem));

    stateAlgaeL2Button.toggleOnTrue(
        new StateAlgaeRemoveL2(
            elevatorSubsystem, wristSubsystem, coralIntakeSubsystem, blingSubsystem));
    stateAlgaeL3Button.toggleOnTrue(
        new StateAlgaeRemoveL3(
            elevatorSubsystem, wristSubsystem, coralIntakeSubsystem, blingSubsystem));

    climbButton.toggleOnTrue(
        new Climb(climberSubsystem, elevatorSubsystem, blingSubsystem, () -> -getElevatorAxis()));
    unClimbButton.onTrue(new UnClimb(climberSubsystem));

    // Auto Cycle Coral Station Controls

    // Enabled state
    enablePathfindingButton.onTrue(autoCycleSubsystem.togglePathfinding());
    pathfindPrecisionAlignToReefButton.toggleOnTrue(
        pathPlannerManager
            .getPathEndPose(autoCycleSubsystem.getState().getSelectedReefBranchPathName())
            .map(endPose -> new DrivePrecisionAlign(swerveDriveSubsystem).getCommand(endPose))
            .orElse(Commands.none()));

    // enableFullAutoPathfindingButton.onTrue(new FullAutoCycle(autoCycleSubsystem).repeatedly());

    cycleCoralStationSideLeftButton.onTrue(autoCycleSubsystem.cycleCoralStationSideLeft());
    cycleCoralStationSideRightButton.onTrue(autoCycleSubsystem.cycleCoralStationSideRight());
    cycleCoralStationPickupPositionLeftButton.onTrue(
        autoCycleSubsystem.cycleCoralStationPositionLeft());
    cycleCoralStationPickupPositionRightButton.onTrue(
        autoCycleSubsystem.cycleCoralStationPositionRight());
    pathfindToSelectedCoralStationButton.toggleOnTrue(
        new PathfindToCoralStation(
            autoCycleSubsystem, () -> DriveCalculator.getChassisSpeeds(swerveDriveSubsystem)));
    // Auto Cycle Reef Branch Controls
    cycleSelectedBranchRightButton.onTrue(autoCycleSubsystem.cycleReefBranchRight());
    cycleSelectedBranchLeftButton.onTrue(autoCycleSubsystem.cycleReefBranchLeft());
    pathfindToSelectedReefBranchButton.toggleOnTrue(
        new PathfindToReef(
            autoCycleSubsystem, () -> DriveCalculator.getChassisSpeeds(swerveDriveSubsystem)));

    // BACKUP -----------------------------------------------------------------------------
    // Coral Intake
    ejectCoralButton.whileTrue(new CoralIntakeEject(coralIntakeSubsystem, () -> EJECT_PERCENTAGE));
    coralIntakeEjectThroughButton.toggleOnTrue(
        new CoralIntakeEjectThrough(coralIntakeSubsystem, () -> EJECT_PERCENTAGE));
    coralIntakeIntakeManualButton.whileTrue(
        new CoralIntakeIntakeManual(coralIntakeSubsystem, () -> INTAKE_PERCENTAGE));
    coralIntakeEjectManualButton.whileTrue(
        new CoralIntakeEjectManual(coralIntakeSubsystem, () -> 0.5));
    // Elevator
    elevatorStowButton.toggleOnTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.CORAL_STOWED));
    elevatorL2Button.toggleOnTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.CORAL_L2_PLACE));
    elevatorL3Button.toggleOnTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.CORAL_L3_PLACE));
    elevatorL4Button.toggleOnTrue(
        new ElevatorRunToHeight(elevatorSubsystem, ElevatorConstants.Setpoints.CORAL_L4_PLACE));
    // Wrist
    wristL4coralButton.toggleOnTrue(
        new WristRunToAngle(wristSubsystem, WristConstants.Setpoints.CORAL_L4_PLACE));
    wristL2and3coralButton.toggleOnTrue(
        new WristRunToAngle(wristSubsystem, WristConstants.Setpoints.CORAL_L3_PLACE));
    wristGroundButton.toggleOnTrue(
        new WristRunToAngle(wristSubsystem, WristConstants.Setpoints.CORAL_GROUND_INTAKE));
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
