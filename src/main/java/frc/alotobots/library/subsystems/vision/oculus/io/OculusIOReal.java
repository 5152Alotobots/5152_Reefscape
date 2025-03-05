package frc.alotobots.library.subsystems.vision.oculus.io;

import edu.wpi.first.networktables.*;

/** Implementation of OculusIO for real hardware communication via NetworkTables. */
public class OculusIOReal implements OculusIO {
  /** NetworkTable for Oculus communication */
  private final NetworkTable nt4Table;

  /** Subscriber for MISO (Master In Slave Out) values */
  private final IntegerSubscriber questMiso;

  /** Publisher for MOSI (Master Out Slave In) values */
  private final IntegerPublisher questMosi;

  /** Subscriber for frame count updates */
  private final IntegerSubscriber questFrameCount;

  /** Subscriber for timestamp updates */
  private final DoubleSubscriber questTimestamp;

  /** Subscriber for position updates */
  private final FloatArraySubscriber questPosition;

  /** Subscriber for quaternion orientation updates */
  private final FloatArraySubscriber questQuaternion;

  /** Subscriber for Euler angle updates */
  private final FloatArraySubscriber questEulerAngles;

  /** Subscriber for battery percentage updates */
  private final DoubleSubscriber questBatteryPercent;

  /** Subscriber for Quest heartbeat counter */
  private final DoubleSubscriber questHeartbeat;

  /** Subscriber for Quest connection status */
  private final IntegerSubscriber connectionStatus;

  /** Publisher for robot heartbeat counter */
  private final DoublePublisher robotHeartbeat;

  /** Publisher for pose reset commands */
  private final DoubleArrayPublisher resetPosePub;

  /**
   * Creates a new OculusIOReal instance and initializes all NetworkTable publishers and
   * subscribers.
   */
  public OculusIOReal() {
    nt4Table = NetworkTableInstance.getDefault().getTable("questnav");
    questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    questMosi = nt4Table.getIntegerTopic("mosi").publish();
    questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(-1);
    questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(-1.0);
    questPosition =
            nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questQuaternion =
            nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    questEulerAngles =
            nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(-1.0);
    questHeartbeat = nt4Table.getDoubleTopic("questHeartbeat").subscribe(0.0);
    connectionStatus = nt4Table.getIntegerTopic("connectionStatus").subscribe(0);

    robotHeartbeat = nt4Table.getDoubleTopic("robotHeartbeat").publish();
    resetPosePub = nt4Table.getDoubleArrayTopic("resetpose").publish();
  }

  @Override
  public void updateInputs(OculusIOInputs inputs) {
    inputs.position = questPosition.get();
    inputs.quaternion = questQuaternion.get();
    inputs.eulerAngles = questEulerAngles.get();
    inputs.timestamp = questTimestamp.get();
    inputs.frameCount = (int) questFrameCount.get();
    inputs.batteryPercent = questBatteryPercent.get();
    inputs.misoValue = (int) questMiso.get();
    inputs.questHeartbeat = questHeartbeat.get();
    inputs.connectionStatus = (int) connectionStatus.get();
  }

  @Override
  public void setMosi(int value) {
    questMosi.set(value);
  }

  @Override
  public void setResetPose(double x, double y, double rotation) {
    resetPosePub.set(new double[] {x, y, rotation});
  }

  @Override
  public void sendHeartbeat(double value) {
    robotHeartbeat.set(value);
  }
}