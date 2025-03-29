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
package frc.alotobots.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import lombok.experimental.UtilityClass;

@UtilityClass
public final class NotificationPresets {

  public final class Auto {
    public static void sendAutoPathChangeNotification(String pathName) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Path Change",
              "The path has changed to: " + pathName,
              3000));
    }
  }

  public final class AprilTag {
    public static void sendAprilTagCameraDisconnectedNotification(String cameraName) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Camera Disconnected",
              cameraName + " has been disconnected.",
              3000));
    }
  }

  public final class Oculus {
    public static void sendOculusPoseResetNotification(Pose2d newPose) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Oculus Pose Reset",
              "New Position: "
                  + newPose.getTranslation().toString()
                  + newPose.getRotation().getDegrees()
                  + "deg",
              3000));
    }

    public static void sendOculusTransformUpdateNotification(Transform2d newTransform) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Oculus Transform Update",
              "New Position: "
                  + newTransform.getTranslation().toString()
                  + newTransform.getRotation().getDegrees()
                  + "deg",
              3000));
    }

    public static void sendOculusDisconnectedNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Disconnected",
              "The Oculus has been disconnected.",
              3000));
    }

    public static void sendOculusBatteryLowNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Oculus Battery Low",
              "The Oculus battery is low.",
              3000));
    }

    public static void sendOculusBatteryCriticalNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Battery Critical",
              "The Oculus battery is critical.",
              3000));
    }
  }

  public final class SwerveDrive {
    public static void sendSwerveDrivePoseResetNotification(Pose2d newPose) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Swerve Drive Pose Reset",
              "New Position: "
                  + newPose.getTranslation().toString()
                  + newPose.getRotation().getDegrees()
                  + "deg",
              3000));
    }
  }

  public final class General {
    public static void sendGeneralNotification(
        Elastic.ElasticNotification.NotificationLevel level, String title, String message) {
      Elastic.sendAlert(new Elastic.ElasticNotification(level, title, message, 3000));
    }
  }
}
