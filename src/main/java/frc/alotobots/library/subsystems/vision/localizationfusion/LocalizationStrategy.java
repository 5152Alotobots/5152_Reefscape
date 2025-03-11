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
package frc.alotobots.library.subsystems.vision.localizationfusion;

import static edu.wpi.first.units.Units.Milliseconds;
import static frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusionConstants.ELASTIC_NOTIFICATION_DURATION;
import static frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusionConstants.FALLBACK_SOURCE_STATES;
import static frc.alotobots.library.subsystems.vision.localizationfusion.LocalizationFusionConstants.Logging.AKIT_SOURCE_CHANGED_PATH;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.alotobots.util.Elastic;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

/**
 * Manages a prioritized list of localization sources and selects the highest priority source that's
 * not in a FAULT state.
 */
public class LocalizationStrategy {
  private final List<LocalizationSource> prioritizedSources;
  private LocalizationSource currentActiveSource;
  private BiConsumer<Pose2d, Matrix<N3, N1>> poseConsumer;

  /**
   * Creates a LocalizationStrategy with sources in priority order (highest priority first).
   *
   * @param sources Localization sources in descending priority order
   */
  public LocalizationStrategy(LocalizationSource... sources) {
    this.prioritizedSources = Arrays.asList(sources);
    this.currentActiveSource = null;
  }

  /**
   * Updates all sources and selects the highest priority non-FAULT source. Sends the current pose
   * to the registered consumer if available.
   */
  public void update() {
    // Update all sources
    for (LocalizationSource source : prioritizedSources) {
      source.update();
    }

    // Store previous active source for transition detection
    LocalizationSource previousActiveSource = currentActiveSource;

    // Select highest priority non-FAULT source
    currentActiveSource = null;
    for (LocalizationSource source : prioritizedSources) {
      if (!FALLBACK_SOURCE_STATES.contains(source.getState())) {
        currentActiveSource = source;
        break;
      }
    }

    // Log source transition if it occurred
    if (previousActiveSource != currentActiveSource) {
      logSourceChange(previousActiveSource, currentActiveSource);
    }

    // Send pose to consumer if available
    if (poseConsumer != null && currentActiveSource != null) {
      poseConsumer.accept(
          currentActiveSource.getPose(), currentActiveSource.getStandardDeviations());
    }
  }

  /**
   * @return The current pose from the active source, or null if no valid source
   */
  public Pose2d getCurrentPose() {
    return (currentActiveSource != null) ? currentActiveSource.getPose() : null;
  }

  /**
   * @return The standard deviations from the active source, or null if no valid source
   */
  public Matrix<N3, N1> getCurrentStdDevs() {
    return (currentActiveSource != null) ? currentActiveSource.getStandardDeviations() : null;
  }

  /**
   * @return The currently active localization source, or null if none available
   */
  public LocalizationSource getCurrentSource() {
    return currentActiveSource;
  }

  /**
   * @return The state of the current active source, or FAULT if no valid source
   */
  public LocalizationSourceState getStrategyState() {
    return (currentActiveSource != null)
        ? currentActiveSource.getState()
        : LocalizationSourceState.FAULT;
  }

  /**
   * Registers a consumer to receive pose updates. Typically this would be the
   * swerveDrive.addVisionMeasurement method.
   *
   * @param poseConsumer Consumer that accepts a pose and standard deviations
   */
  public void registerPoseConsumer(BiConsumer<Pose2d, Matrix<N3, N1>> poseConsumer) {
    this.poseConsumer = poseConsumer;
  }

  private void logSourceChange(LocalizationSource previousSource, LocalizationSource newSource) {
    String description =
        String.format(
            "Localization source changed from %s to %s",
            previousSource != null ? previousSource.getType() : "None",
            newSource != null ? newSource.getType() : "None");

    Elastic.sendAlert(
        new Elastic.ElasticNotification(
            Elastic.ElasticNotification.NotificationLevel.INFO,
            "Localization source changed",
            description,
            (int) ELASTIC_NOTIFICATION_DURATION.in(Milliseconds)));
    Logger.recordOutput(AKIT_SOURCE_CHANGED_PATH, description);
  }
}
