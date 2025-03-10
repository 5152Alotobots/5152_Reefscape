package frc.alotobots.library.subsystems.vision.localizationfusion.util;

import java.util.List;
import java.util.Map;

/**
 * Interface for different strategies to fuse or select poses
 */
interface PoseFusionStrategy {
    String getStrategyName();
    PoseSourceResult calculateFusedPose(List<PoseSource> sources);
    Map<PoseSource, Double> getSourceContributions(); // Shows how much each source contributed
}