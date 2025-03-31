package frc.alotobots.library.subsystems.vision.oculus.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.IntegerPublisher;
import org.littletonrobotics.junction.Logger;

/**
 * Handler for retrying Quest commands with proper delays between attempts.
 */
public class QuestCommandRetryHandler {
  private final IntegerPublisher questMosi;

  private int maxAttempts;
  private int currentAttempt = 0;
  private int command;
  private int expectedResponse;
  private boolean isActive = false;

  // Timing variables now as instance variables
  private final double commandDelaySeconds;
  private final double retryDelaySeconds;
  private double lastCommandTime = 0;
  private boolean clearSent = false;
  private boolean commandSent = false;

  private Runnable onSuccessCallback = null;
  private Runnable onFailureCallback = null;

  /**
   * Creates a new command retry handler with default timing values
   *
   * @param questMosi MOSI publisher for sending commands
   */
  public QuestCommandRetryHandler(IntegerPublisher questMosi) {
    this(questMosi, 0.05, 0.2);
  }

  /**
   * Creates a new command retry handler with custom timing values
   *
   * @param questMosi MOSI publisher for sending commands
   * @param commandDelaySeconds Delay between clear and command (seconds)
   * @param retryDelaySeconds Delay between retry attempts (seconds)
   */
  public QuestCommandRetryHandler(
          IntegerPublisher questMosi,
          double commandDelaySeconds,
          double retryDelaySeconds) {
    this.questMosi = questMosi;
    this.commandDelaySeconds = commandDelaySeconds;
    this.retryDelaySeconds = retryDelaySeconds;
  }

  /**
   * Start a new command retry sequence
   */
  public void startRetry(
          int maxAttempts,
          int command,
          int expectedResponse,
          int clearCommand,
          Runnable onSuccess,
          Runnable onFailure) {

    this.maxAttempts = maxAttempts;
    this.command = command;
    this.expectedResponse = expectedResponse;
    this.currentAttempt = 0;
    this.isActive = true;
    this.onSuccessCallback = onSuccess;
    this.onFailureCallback = onFailure;

    // Reset state
    clearSent = false;
    commandSent = false;
    lastCommandTime = Timer.getTimestamp();
  }

  /**
   * Simplified method that allows skipping callbacks
   */
  public void startRetry(int maxAttempts, int command, int expectedResponse, int clearCommand) {
    startRetry(maxAttempts, command, expectedResponse, clearCommand, null, null);
  }

  /**
   * Update method to be called in updateInputs
   *
   * @param currentResponse Current response value
   * @param clearCommand Command to clear previous commands
   */
  public void update(int currentResponse, int clearCommand) {
    if (!isActive) return;

    double currentTime = Timer.getTimestamp();

    // Check if command succeeded
    if (currentResponse == expectedResponse) {
      isActive = false;
      Logger.recordOutput("Oculus/Log", "Command succeeded on attempt " + currentAttempt);

      // Execute success callback if provided
      if (onSuccessCallback != null) {
        onSuccessCallback.run();
      }
      return;
    }

    // State machine for sending commands with delays
    if (!clearSent) {
      // Send clear command first
      questMosi.set(clearCommand);
      clearSent = true;
      lastCommandTime = currentTime;
      Logger.recordOutput("Oculus/Log", "Sent clear command");
      return;
    }

    if (!commandSent && (currentTime - lastCommandTime) > commandDelaySeconds) {
      // After delay, send the actual command
      questMosi.set(command);
      commandSent = true;
      lastCommandTime = currentTime;
      currentAttempt++;
      Logger.recordOutput("Oculus/Log", "Sent command, attempt " + currentAttempt + " of " + maxAttempts);
      return;
    }

    // If we've reached max attempts, mark as failed
    if (currentAttempt >= maxAttempts && commandSent) {
      isActive = false;
      Logger.recordOutput("Oculus/Log", "Command failed after " + maxAttempts + " attempts");

      // Execute failure callback if provided
      if (onFailureCallback != null) {
        onFailureCallback.run();
      }
      return;
    }

    // Check if it's time for the next retry
    if (commandSent && (currentTime - lastCommandTime) > retryDelaySeconds) {
      // Reset for next attempt
      clearSent = false;
      commandSent = false;
      // Next attempt will start on next update call
      Logger.recordOutput("Oculus/Log", "Preparing retry " + (currentAttempt+1) + " of " + maxAttempts);
    }
  }

  /**
   * Returns whether a command is currently being executed
   */
  public boolean isCommandActive() {
    return isActive;
  }
}