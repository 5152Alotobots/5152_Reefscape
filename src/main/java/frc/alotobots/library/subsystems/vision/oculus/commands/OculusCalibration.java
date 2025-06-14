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
package frc.alotobots.library.subsystems.vision.oculus.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.vision.oculus.OculusSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class OculusCalibration {
  // -- Calculate Oculus Offset (copied from
  // https://github.com/FRC5010/Reefscape2025/blob/main/TigerShark2025/src/main/java/org/frc5010/common/sensors/camera/QuestNav.java#L65) --

  private Translation2d calculatedOffsetToRobot = Translation2d.kZero;
  private double calculateOffsetCount = 1;

  private Translation2d calculateOffsetToRobot(Pose2d questRobotPose) {
    Rotation2d angle = questRobotPose.getRotation();
    Translation2d displacement = questRobotPose.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  /**
   * When calibrating make sure the rotation in your quest transform is right. (Reality check) If it
   * is non-zero, you may have to swap the x/y and their signs.
   */
  public Command determineOffsetToRobotCenter(
      SwerveDriveSubsystem swerveDriveSubsystem, OculusSubsystem oculusSubsystem) {
    // First reset our pose to 0, 0
    oculusSubsystem.resetPose(Pose2d.kZero);
    Supplier<Pose2d> questPose = oculusSubsystem::getPose;
    return Commands.repeatingSequence(
            Commands.run(
                    () -> {
                      swerveDriveSubsystem.runVelocity(new ChassisSpeeds(0, 0, Math.PI / 10.0));
                    },
                    swerveDriveSubsystem)
                .withTimeout(0.5),
            Commands.runOnce(
                    () -> {
                      // Update current offset
                      Translation2d offset = calculateOffsetToRobot(questPose.get());

                      calculatedOffsetToRobot =
                          calculatedOffsetToRobot
                              .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                              .plus(offset.div(calculateOffsetCount + 1));
                      calculateOffsetCount++;
                      Logger.recordOutput(
                          "OculusCalibration/CalculatedOffset", calculatedOffsetToRobot);
                    })
                .onlyIf(() -> questPose.get().getRotation().getDegrees() > 30))
        .finallyDo(
            () -> {
              // Update current offset
              Translation2d offset = calculateOffsetToRobot(questPose.get());

              calculatedOffsetToRobot =
                  calculatedOffsetToRobot
                      .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                      .plus(offset.div(calculateOffsetCount + 1));
              calculateOffsetCount++;
              Logger.recordOutput("OculusCalibration/CalculatedOffset", calculatedOffsetToRobot);
            });
  }
}
