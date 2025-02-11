package frc.alotobots.reefscape.subsystems.wrist.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

public class WristIOTalonFXConstants {

  public static final int ROTOR_TO_SENSOR_RATIO = 189;
  public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

  public static final double ENCODER_MAGNET_OFFSET = 0;
  public static final SensorDirectionValue ENCODER_DIRCTION_VALUE = SensorDirectionValue.Clockwise_Positive;
}
