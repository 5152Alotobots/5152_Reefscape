package frc.alotobots.library.subsystems.vision.localizationfusion.sources;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import frc.alotobots.library.subsystems.vision.localizationfusion.util.PoseSource;
import frc.alotobots.library.subsystems.vision.localizationfusion.util.PoseSourceResult;
import frc.alotobots.library.subsystems.vision.photonvision.apriltag.AprilTagSubsystem;

import java.util.ArrayDeque;
import java.util.ArrayList;

public class SingleTagPoseSource implements PoseSource {

    private AprilTagSubsystem aprilTagSubsystem;
    private Time lastUpdateTime;

    private ArrayDeque<PoseSourceResult> unreadResults = new ArrayDeque<>(20);

    @Override
    public ArrayList<PoseSourceResult> getUnreadResults() {
        ArrayList<PoseSourceResult> unread = (ArrayList<PoseSourceResult>) unreadResults.stream().toList();
        unreadResults.clear();
        return unread;
    }

    @Override
    public Vector<N3> getStandardDeviations() {
        return null;
    }

    @Override
    public int getPriority() {
        return 0;
    }

    @Override
    public void setPriority(int priority) {

    }

    @Override
    public SourceStatus getStatus() {
        return null;
    }

    @Override
    public double getUpdateFrequency() {
        return 0;
    }

    @Override
    public PoseSourceType getSourceType() {
        return null;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void update() {
    }

    @Override
    public boolean isInitialized() {
        return false;
    }
}
