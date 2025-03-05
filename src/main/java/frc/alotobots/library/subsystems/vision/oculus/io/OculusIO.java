package frc.alotobots.library.subsystems.vision.oculus.io;

import org.littletonrobotics.junction.AutoLog;

/** Interface for handling input/output operations with the Oculus Quest hardware. */
public interface OculusIO {
  /** Data structure for Oculus inputs that can be automatically logged. */
  @AutoLog
  public static class OculusIOInputs {
    /** 3D position coordinates [x, y, z] */
    public float[] position = new float[] {0.0f, 0.0f, 0.0f};

    /** Quaternion orientation [w, x, y, z] */
    public float[] quaternion = new float[] {0.0f, 0.0f, 0.0f, 0.0f};

    /** Euler angles [roll, pitch, yaw] in degrees */
    public float[] eulerAngles = new float[] {0.0f, 0.0f, 0.0f};

    /** Current timestamp from the Oculus */
    public double timestamp = -1.0;

    /** Frame counter from the Oculus */
    public int frameCount = -1;

    /** Battery level percentage */
    public double batteryPercent = -1.0;

    /** Current MISO (Master In Slave Out) value */
    public int misoValue = 0;

    /** Quest heartbeat counter value */
    public double questHeartbeat = 0.0;

    /** Connection status reported by Quest (0=Disconnected, 1=Connecting, 2=Connected, 3=Degraded) */
    public int connectionStatus = 0;
  }

  /**
   * Updates the set of loggable inputs from the Oculus.
   *
   * @param inputs The input object to update with current values
   */
  public default void updateInputs(OculusIOInputs inputs) {}

  /**
   * Sets MOSI (Master Out Slave In) value for Quest communication.
   *
   * @param value The MOSI value to set
   */
  public default void setMosi(int value) {}

  /**
   * Sets the pose components for resetting the Oculus position tracking.
   *
   * @param x The X coordinate
   * @param y The Y coordinate
   * @param rotation The rotation in degrees
   */
  public default void setResetPose(double x, double y, double rotation) {}

  /**
   * Publishes robot heartbeat to maintain connection with Quest.
   *
   * @param value Incrementing heartbeat counter
   */
  public default void sendHeartbeat(double value) {}
}