package frc.alotobots.reefscape.subsystems.elevator.constants;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

/**
 * Constants for the simulated elevator subsystem using TalonFX motors.
 * Contains PID constants for different game piece modes and physical configuration parameters.
 */
public class ElevatorTalonFXSimConstants {
    /**
     * TalonFX-specific PID and motion control constants for coral game piece (Coral mode).
     */
    public static final class CoralPIDConstants {
        /** Position control proportional gain */
        public static final double KP = 0.1;

        /** Position control integral gain */
        public static final double KI = 0.0;

        /** Position control derivative gain */
        public static final double KD = 0.0;

        /** Acceleration feedforward gain */
        public static final double KA = 0.0;

        /** Gravity compensation gain */
        public static final double KG = 0.0;

        /** Static friction compensation */
        public static final double KS = 0.0;

        /** Velocity feedforward gain */
        public static final double KV = 0.0;
    }

    /**
     * TalonFX-specific PID and motion control constants for algae game piece (Algae mode).
     */
    public static final class AlgaePIDConstants {
        /** Position control proportional gain */
        public static final double KP = 0.1;

        /** Position control integral gain */
        public static final double KI = 0.0;

        /** Position control derivative gain */
        public static final double KD = 0.0;

        /** Acceleration feedforward gain */
        public static final double KA = 0.0;

        /** Gravity compensation gain */
        public static final double KG = 0.0;

        /** Static friction compensation */
        public static final double KS = 0.0;

        /** Velocity feedforward gain */
        public static final double KV = 0.0;
    }

    /**
     * TalonFX-specific hardware configuration constants defining the mechanical properties of the
     * elevator system.
     */
    public static final class HardwareConfig {
        /** Gear ratio between motor and pulley (motor rotations : pulley rotations) */
        public static final double GEAR_RATIO = 35.0;

        /** Weight of full assembly that is being lifted */
        public static final Mass MASS = Pounds.of(5.2);
    }

    /**
     * Physical dimensions and mechanical configuration of the elevator system.
     */
    public static final class Mechanics {
        /** Diameter of the elevator pulley */
        public static final Distance PULLEY_DIAMETER = Centimeters.of(3.2);

        /** Circumference of the elevator pulley */
        public static final Distance PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);
    }
}