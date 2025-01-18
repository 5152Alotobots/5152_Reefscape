# Pathfind to Selected Reef Branch Command

Initiates pathfinding to navigate the robot to the currently selected reef branch position. This command automatically transitions to precision alignment when near the target.

## Overview
The command handles the complete movement sequence:
1. Initiates pathfinding to selected reef branch
2. Monitors progress toward target
3. Transitions to precision alignment when close enough
4. Completes when aligned within tolerance

## Required Subsystems
- AutoCycle Subsystem
- Swerve Drive Subsystem
- PathPlanner Manager

## Constructor/Factory Method
```java
pathfindToSelectedReefBranchPathName()
```
Called from AutoCycle subsystem instance. No direct parameters as it uses the subsystem's current state.

## Configuration Requirements
Requires proper configuration of:
- PathPlanner paths named `BranchApproach_[BRANCH]_[LEVEL]`
- Translation and rotation tolerances in AutoCycle constants
- SwerveDrive alignment parameters

## JavaDoc Reference
Full documentation at [Command JavaDoc](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/autocycle/commands/package-summary.html)
