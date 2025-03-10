package frc.alotobots.library.subsystems.vision.localizationfusion.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Base interface for all pose estimation sources
 */
/**
 * Interface representing a source of pose estimation data.
 */
public interface PoseSource {
    /**
     * Gets the unread results of the pipeline. Automatically clears the buffer after this method is used.
     * Ensure it is called ONCE each loop to avoid data loss.
     *
     * @return the unread poses as an object
     */
    ArrayList<PoseSourceResult> getUnreadResults();

    // Uncertainty using standard deviations
    /**
     * Gets the standard deviation in X/Y/Rot positions.
     *
     * @return the standard deviation in X/Y/Rot positions
     */
    Matrix<N3, N1> getStandardDeviations();

    /**
     * Sets if the pose source is connected to the robot.
     * */
    void isConnected(boolean isConnected);

    // Priority system
    /**
     * Gets the priority of the pose source.
     *
     * @return the priority, where a lower number indicates higher priority
     */
    int getPriority();
    /**
     * Sets the priority of the pose source.
     *
     * @param priority the priority to set, where a lower number indicates higher priority
     */
    void setPriority(int priority);
    
    // Status information
    enum SourceStatus { READY, UNINITIALIZED, DEGRADED, OFFLINE }
    /**
     * Gets the status of the pose source.
     *
     * @return the status of the pose source
     */
    SourceStatus getStatus();

    enum PoseSourceType { MULTI_TAG, SINGLE_TAG, OCULUS, ODOMETRY }

    // Metadata
    /**
     * Gets the type of the pose source.
     *
     * @return the type of the pose source, e.g., "Odometry", "Vision", "IMU", etc.
     */
    PoseSourceType getSourceType();
    
    // Lifecycle methods
    /**
     * Initializes the pose source.
     */
    void initialize();
    /**
     * Updates the pose source.
     */
    void update(Supplier<List<PoseSourceResult>> resultSupplier);
    /**
     * Checks if the pose source is initialized.
     *
     * @return true if the pose source is initialized, false otherwise
     */
    boolean isInitialized();
}