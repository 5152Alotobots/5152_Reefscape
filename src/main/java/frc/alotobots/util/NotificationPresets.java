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

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

  /** Contains notification methods related to the QuestNav tracking system. */
  public final class QuestNav {
    /** Tracks whether a battery low notification has already been sent */
    private static boolean questNavBatteryLowNotificationSent = false;

    /** Tracks whether a battery critical notification has already been sent */
    private static boolean questNavBatteryCriticalNotificationSent = false;

    /** Tracks whether a disconnection notification has already been sent */
    private static boolean questNavDisconnectedNotificationSent = false;

    /** Tracks whether a tracking lost notification has already been sent */
    private static boolean questNavTrackingLostNotificationSent = false;

    /** Sends a notification when the QuestNav pose reset fails. */
    public static void sendQuestNavPoseResetFailedNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "QuestNav Pose Reset Failed",
              "Not using Quest",
              3000));
    }

    /**
     * Sends a notification when the QuestNav pose is reset.
     *
     * @param newPose The new pose after reset
     */
    public static void sendQuestNavPoseResetNotification(Pose3d newPose) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "QuestNav Pose Reset",
              "New Position: "
                  + newPose.getTranslation().toString()
                  + newPose.getRotation().getMeasureZ().in(Degrees)
                  + "deg",
              3000));
    }

    /** Sends a notification when the QuestNav heading reset fails. */
    public static void sendQuestNavHeadingResetFailedNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "QuestNav Heading Reset Failed",
              "Not using Quest",
              3000));
    }

    /** Sends a notification when the QuestNav heading is reset. */
    public static void sendQuestNavHeadingResetNotification() {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "QuestNav Heading Reset",
              "Reset complete",
              3000));
    }

    /**
     * Sends a notification when the QuestNav transform is updated.
     *
     * @param newTransform The new transform applied to the QuestNav
     */
    public static void sendQuestNavTransformUpdateNotification(Transform2d newTransform) {
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "QuestNav Transform Update",
              "New Position: "
                  + newTransform.getTranslation().toString()
                  + newTransform.getRotation().getDegrees()
                  + "deg",
              3000));
    }

    /**
     * Sends a notification when the QuestNav reconnects. Only sends if a disconnection was
     * previously reported.
     */
    public static void sendQuestNavReconnectedNotification() {
      // Don't send the notification if the QuestNav has never been disconnected
      if (!questNavDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.INFO,
              "QuestNav Reconnected",
              "The QuestNav has been reconnected.",
              3000));
      questNavDisconnectedNotificationSent = false;
    }

    /**
     * Sends a notification when the QuestNav disconnects. Only sends the notification once until
     * the device reconnects.
     */
    public static void sendQuestNavDisconnectedNotification() {
      // Only send the notification once
      if (questNavDisconnectedNotificationSent) return;

      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "QuestNav Disconnected",
              "The QuestNav has been disconnected.",
              3000));
      questNavDisconnectedNotificationSent = true;
    }

    /**
     * Sends a notification when the QuestNav battery is low. Only sends the notification once per
     * battery discharge cycle.
     */
    public static void sendQuestNavBatteryLowNotification() {
      // Only send the notification once
      if (questNavBatteryLowNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "QuestNav Battery Low",
              "The QuestNav battery is low.",
              3000));
      questNavBatteryLowNotificationSent = true;
    }

    /**
     * Sends a notification when the QuestNav battery is critically low. Only sends the notification
     * once per battery discharge cycle.
     */
    public static void sendQuestNavBatteryCriticalNotification() {
      // Only send the notification once
      if (questNavBatteryCriticalNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.ERROR,
              "QuestNav Battery Critical",
              "The QuestNav battery is critical.",
              3000));
      questNavBatteryCriticalNotificationSent = true;
    }

    /**
     * Sends a notification when the QuestNav tracking is lost. Only sends the notification once
     * until tracking is regained.
     *
     * @param totalTrackingLostEvents The times the quest has lost tracking total since app boot.
     */
    public static void sendQuestNavTrackingLostNotification(int totalTrackingLostEvents) {
      // Only send the notification once
      if (questNavTrackingLostNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "QuestNav Tracking Lost",
              String.format(
                  "QuestNav Tracking Lost. (%d time(s) this boot)", totalTrackingLostEvents),
              3000));
      questNavTrackingLostNotificationSent = true;
    }

    /**
     * Sends a notification when the QuestNav tracking is regained. Only sends the notification once
     * until tracking is lost again.
     */
    public static void sendQuestNavTrackingRegainedNotification() {
      // Don't send the notification if the tracking has never been lost
      if (!questNavTrackingLostNotificationSent) return;
      Elastic.sendAlert(
          new Elastic.ElasticNotification(
              Elastic.ElasticNotification.NotificationLevel.WARNING,
              "QuestNav Tracking Regained",
              "QuestNav Tracking Regained. Switching back to QuestNav tracking.",
              3000));
      questNavTrackingLostNotificationSent = false;
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
