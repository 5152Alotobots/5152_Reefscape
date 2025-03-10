package frc.alotobots.library.subsystems.vision.localizationfusion.util;

/**
 * Interface for condition-based priority adjustments
 */
interface PriorityModifier {
    String getModifierName();
    int calculatePriorityAdjustment(PoseSource source);
    boolean isActive();
}