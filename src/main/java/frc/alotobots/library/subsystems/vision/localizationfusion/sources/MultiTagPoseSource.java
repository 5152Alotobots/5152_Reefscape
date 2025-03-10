package frc.alotobots.library.subsystems.vision.localizationfusion.sources;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.alotobots.library.subsystems.vision.localizationfusion.util.PoseSource;
import frc.alotobots.library.subsystems.vision.localizationfusion.util.PoseSourceResult;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static frc.alotobots.library.subsystems.vision.localizationfusion.constants.PoseSourcePriorities.MULTI_TAG_PRIORITY;

public class MultiTagPoseSource implements PoseSource {

    private final AprilTagSubsystem aprilTagSubsystem;
    private boolean initialized = false;
    private int priority = MULTI_TAG_PRIORITY; // Higher priority than single tag (lower number = higher priority)
    private Matrix<N3, N1> standardDeviations;

    private final ArrayDeque<PoseSourceResult> unreadResults = new ArrayDeque<>(20);

    public MultiTagPoseSource(AprilTagSubsystem aprilTagSubsystem) {
        this.aprilTagSubsystem = aprilTagSubsystem;
    }

    @Override
    public ArrayList<PoseSourceResult> getUnreadResults() {
        ArrayList<PoseSourceResult> unread = new ArrayList<>(unreadResults);
        unreadResults.clear();
        return unread;
    }

    @Override
    public Matrix<N3, N1> getStandardDeviations() {
        return standardDeviations;
    }

    @Override
    public void isConnected(boolean isConnected) {

    }

    @Override @AutoLogOutput(key = "PoseSource/MultiTag/Priority")
    public int getPriority() {
        return priority;
    }

    @Override
    public void setPriority(int priority) {
        this.priority = priority;
    }

    @Override @AutoLogOutput(key = "PoseSource/MultiTag/Status")
    public SourceStatus getStatus() {
        if (!aprilTagSubsystem.isConnected()) {
            return SourceStatus.OFFLINE;
        }

        if (!initialized) {
            return SourceStatus.UNINITIALIZED;
        }

        return SourceStatus.READY;
    }

    @Override
    public PoseSourceType getSourceType() {
        return PoseSourceType.MULTI_TAG;
    }

    @Override
    public void initialize() {
        initialized = true;
    }

    @Override
    public void update(Supplier<List<PoseSourceResult>> supplier) {
        if (!initialized) {
            return;
        }
        List<PoseSourceResult> results = supplier.get();
        if (results.isEmpty()) {
            return;
        }
        unreadResults.addAll(results);
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }
}