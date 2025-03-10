package frc.alotobots.library.subsystems.vision.localizationfusion.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public record PoseSourceResult(Time timestamp, Matrix<N3, N1> standardDevations, PoseSource.PoseSourceType poseSourceType, Pose3d pose) {

}
