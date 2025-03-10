package frc.alotobots.library.subsystems.vision.localizationfusion.constants;

import lombok.experimental.UtilityClass;

/**
 * This class contains constants for pose source priorities.
 * The priorities are used to determine the order in which pose sources are considered during fusion.
 * Lower values indicate higher priority.
 */
@UtilityClass
public final class PoseSourcePriorities {
    public static final int MULTI_TAG_PRIORITY = 1;
    public static final int SINGLE_TAG_PRIORITY = 2;
    public static final int OCULUS_PRIORITY = 3;
}
