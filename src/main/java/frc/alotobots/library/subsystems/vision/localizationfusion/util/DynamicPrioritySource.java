package frc.alotobots.library.subsystems.vision.localizationfusion.util;

import java.util.List;

/**
 * Extension that adds dynamic priority adjustment capabilities
 */
interface DynamicPrioritySource extends PoseSource {
    // Base priority configuration
    int getBasePriority();
    void setBasePriority(int priority);
    
    // Dynamic priority calculation
    int calculateEffectivePriority();
    
    // Condition-based modifiers
    void addPriorityModifier(PriorityModifier modifier);
    void removePriorityModifier(PriorityModifier modifier);
    List<PriorityModifier> getActiveModifiers();
}