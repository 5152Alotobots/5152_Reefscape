# AutoCycle Subsystem

The AutoCycle subsystem manages automated navigation and positioning of the robot during matches. It handles the selection and targeting of different reef branches, levels, and coral station positions, enabling precise movement patterns and game piece collection.

## Core Components

### State Management
The subsystem maintains its state through the `AutoCycleState` class, which tracks:
- Current reef branch selection
- Current reef level selection
- Current coral station side
- Current coral station pickup position
- Active pathfinding status

### Constructor Parameters

```java
public AutoCycleSubsystem(PathPlannerManager pathPlannerManager,
                         SwerveDriveSubsystem swerveDriveSubsystem)
```

- `pathPlannerManager`: Handles path generation and following capabilities
- `swerveDriveSubsystem`: Controls robot movement and positioning

## Commands that use this subsystem

The AutoCycle subsystem provides several command factories for robot control:

### Navigation Commands
- `pathfindToSelectedReefBranchPathName()`: Creates command to navigate to selected reef branch
- `pathfindToSelectedCoralStationPathName()`: Creates command to navigate to selected coral station
- `stopPathfinding()`: Creates command to stop active pathfinding

### Selection Commands
- `runCycleReefBranchRight()/Left()`: Cycle reef branch selection
- `runCycleReefLevelUp()/Down()`: Cycle reef level selection
- `runCycleCoralStationSideRight()/Left()`: Cycle coral station side selection
- `runCycleCoralStationPickupPositionRight()/Left()`: Cycle pickup position selection
- Direct setters for all selections through `runSetReefBranch()`, `runSetReefLevel()`, etc.

## Configuration Requirements

### Constants
The following must be configured in `AutoCycleConstants`:
- `ALIGNMENT_TRANSLATION_TOLERANCE`: Translation tolerance in meters - determines how close the robot needs to be to its target position before switching from pathfinding to precision alignment (default 0.2 meters)
- `ALIGNMENT_ROTATION_TOLERANCE`: Rotation tolerance in radians - specifies the maximum angular difference allowed before the robot is considered properly aligned with its target orientation (default 15 degrees)

### PathPlanner Requirements
Path files must be created with names following these patterns in PathPlanner:
- Reef branches: `BranchApproach_[BRANCH]_[LEVEL]` - for example, "BranchApproach_A_L2" creates a path to branch A at level 2
- Coral stations: `CoralStationApproach_[SIDE]_[POSITION]` - for example, "CoralStationApproach_RIGHT_P1" creates a path to position 1 on the right side of the coral station

### Required Subsystems
- Must have functioning `SwerveDriveSubsystem` instance
- Must have configured `PathPlannerManager` instance

## JavaDoc Reference
Full JavaDoc documentation can be found at:
- [AutoCycle Subsystem](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/autocycle/package-summary.html)
- [AutoCycle State](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/autocycle/util/package-summary.html)
- [AutoCycle Constants](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/autocycle/constants/package-summary.html)

## Related Links
- [Swerve Drive Subsystem](/5152_Reefscape/library/subsystems/swerve)
- [Drive Precision Align Command](/5152_Reefscape/library/commands/swerve/driveprecisionalign)
- [Field Constants](/5152_Reefscape/game/constants/field)
