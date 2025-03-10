package frc.alotobots.library.subsystems.vision.localizationfusion.util;

import java.util.List;

/**
 * Registry to manage available pose sources
 */
interface PoseSourceRegistry {
    void registerSource(PoseSource source);
    void unregisterSource(PoseSource source);
    List<PoseSource> getAllSources();
    List<PoseSource> getActiveSources();
    List<PoseSource> getSourcesByPriority(int priority);
    List<PoseSource> getHighestPrioritySources();
    PoseSource getSourceByType(PoseSource.PoseSourceType type);
}