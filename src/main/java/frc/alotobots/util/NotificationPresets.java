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

/**
 * Utility class containing preset notification methods for various subsystems and components.
 * Provides standardized alert methods for common events across different robot systems.
 */
@UtilityClass
public final class NotificationPresets {

  /** Contains notification methods related to autonomous mode operations. */
  public final class Auto {
    /**
     * Sends a notification when the autonomous path changes.
     *
     * @param pathName The name of the new path being executed
     */
    public static void sendAutoPathChangeNotification(String pathName) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Path Change",
              "The path has changed to: " + pathName,
              3000));
    }
  }

  /** Contains notification methods related to AprilTag vision system. */
  public final class AprilTag {
    /** Tracks whether a camera disconnected notification has already been sent */
    private static boolean aprilTagCameraDisconnectedNotificationSent = false;

    /**
     * Sends a notification when an AprilTag camera disconnects. Only sends the notification once
     * until the camera is reconnected.
     *
     * @param cameraName The name of the disconnected camera
     */
    public static void sendAprilTagCameraDisconnectedNotification(String cameraName) {
      // Only send the notification once
      if (aprilTagCameraDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Camera Disconnected",
              cameraName + " has been disconnected.",
              3000));
      aprilTagCameraDisconnectedNotificationSent = true;
    }

    /**
     * Sends a notification when an AprilTag camera reconnects. Only sends if a disconnection was
     * previously reported.
     *
     * @param cameraName The name of the reconnected camera
     */
    public static void sendAprilTagCameraReconnectedNotification(String cameraName) {
      // Don't send the notification if the camera has never been disconnected
      if (!aprilTagCameraDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Camera Reconnected",
              cameraName + " has been reconnected.",
              3000));
      aprilTagCameraDisconnectedNotificationSent = false;
    }
  }

  /** Contains notification methods related to the Oculus tracking system. */
  public final class Oculus {
    /** Tracks whether a battery low notification has already been sent */
    private static boolean oculusBatteryLowNotificationSent = false;

    /** Tracks whether a battery critical notification has already been sent */
    private static boolean oculusBatteryCriticalNotificationSent = false;

    /** Tracks whether a disconnection notification has already been sent */
    private static boolean oculusDisconnectedNotificationSent = false;

    /** Tracks whether a tracking lost notification has already been sent */
    private static boolean oculusTrackingLostNotificationSent = false;

    /**
     * Sends a notification when the Oculus pose reset fails.
     */
    public static void sendOculusPoseResetFailedNotification() {
      Elastic.sendAlert(
              new Elastic.ElasticNotification(
                      Elastic.ElasticNotification.NotificationLevel.ERROR,
                      "Oculus Pose Reset Failed",
                      "Not using Quest",
                      3000));
    }

    /**
     * Sends a notification when the Oculus pose is reset.
     *
     * @param newPose The new pose after reset
     */
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

    /**
     * Sends a notification when the Oculus heading reset fails.
     */
    public static void sendOculusHeadingResetFailedNotification() {
      Elastic.sendAlert(
              new Elastic.ElasticNotification(
                      Elastic.ElasticNotification.NotificationLevel.ERROR,
                      "Oculus Heading Reset Failed",
                      "Not using Quest",
                      3000));
    }

    /**
     * Sends a notification when the Oculus heading is reset.
     */
    public static void sendOculusHeadingResetNotification() {
      Elastic.sendAlert(
              new Elastic.ElasticNotification(
                      Elastic.ElasticNotification.NotificationLevel.INFO,
                      "Oculus Heading Reset",
                      "Reset complete",
                      3000));
    }

    /**
     * Sends a notification when the Oculus transform is updated.
     *
     * @param newTransform The new transform applied to the Oculus
     */
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

    /**
     * Sends a notification when the Oculus reconnects. Only sends if a disconnection was previously
     * reported.
     */
    public static void sendOculusReconnectedNotification() {
      // Don't send the notification if the Oculus has never been disconnected
      if (!oculusDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "Oculus Reconnected",
              "The Oculus has been reconnected.",
              3000));
      oculusDisconnectedNotificationSent = false;
    }

    /**
     * Sends a notification when the Oculus disconnects. Only sends the notification once until the
     * device reconnects.
     */
    public static void sendOculusDisconnectedNotification() {
      // Only send the notification once
      if (oculusDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Disconnected",
              "The Oculus has been disconnected.",
              3000));
      oculusDisconnectedNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus battery is low. Only sends the notification once per
     * battery discharge cycle.
     */
    public static void sendOculusBatteryLowNotification() {
      // Only send the notification once
      if (oculusBatteryLowNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Oculus Battery Low",
              "The Oculus battery is low.",
              3000));
      oculusBatteryLowNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus battery is critically low. Only sends the notification
     * once per battery discharge cycle.
     */
    public static void sendOculusBatteryCriticalNotification() {
      // Only send the notification once
      if (oculusBatteryCriticalNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "Oculus Battery Critical",
              "The Oculus battery is critical.",
              3000));
      oculusBatteryCriticalNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus tracking is lost. Only sends the notification once until
     * tracking is regained.
     *
     * @param totalTrackingLostEvents The times the quest has lost tracking total since app boot.
     */
    public static void sendOculusTrackingLostNotification(int totalTrackingLostEvents) {
      // Only send the notification once
      if (oculusTrackingLostNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Oculus Tracking Lost",
              String.format(
                  "Oculus Tracking Lost. (%d time(s) this boot)", totalTrackingLostEvents),
              3000));
      oculusTrackingLostNotificationSent = true;
    }

    /**
     * Sends a notification when the Oculus tracking is regained. Only sends the notification once
     * until tracking is lost again.
     */
    public static void sendOculusTrackingRegainedNotification() {
      // Don't send the notification if the tracking has never been lost
      if (!oculusTrackingLostNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "Oculus Tracking Regained",
              "Oculus Tracking Regained. Switching back to Oculus tracking.",
              3000));
      oculusTrackingLostNotificationSent = false;
    }
  }

  /** Contains notification methods related to the Swerve Drive subsystem. */
  public final class SwerveDrive {
    /**
     * Sends a notification when the Swerve Drive pose is reset.
     *
     * @param newPose The new pose after reset
     */
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

  /** Contains general-purpose notification methods that can be used by any subsystem. */
  public final class General {
    /**
     * Sends a general notification with customizable parameters.
     *
     * @param level The severity level of the notification
     * @param title The title of the notification
     * @param message The message content of the notification
     */
    public static void sendGeneralNotification(
        Elastic.ElasticNotification.NotificationLevel level, String title, String message) {
      Elastic.sendAlert(new Elastic.ElasticNotification(level, title, message, 3000));
    }
  }
}
