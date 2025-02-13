package frc.alotobots.reefscape.subsystems.coralIntake.io;

import org.littletonrobotics.junction.AutoLog;


import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public interface CoralIntakeIO {
    @AutoLog
    public static class CoralIntakeIOInputs {
        public boolean motorConnected = false;
        
        public LinearVelocity Velocity = MetersPerSecond.zero();
        public Voltage motorAppliedVolts = Volts.zero();
        public Current motorCurrentAmps = Amps.zero();
    }
}
