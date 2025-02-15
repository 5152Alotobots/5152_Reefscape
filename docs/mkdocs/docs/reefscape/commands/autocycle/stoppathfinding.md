# Stop Pathfinding Command

Immediately stops any active pathfinding operation. This command clears all pathfinding states and halts current path following.

## Overview
The command:
1. Sets pathfinding flags to false
2. Clears active path name
3. Completes immediately after stopping

## Required Subsystems
- AutoCycle Subsystem

## Constructor/Factory Method
```java
stopPathfinding()
```
Called from AutoCycle subsystem instance. No parameters required.

## Configuration Requirements
No additional configuration required.

## JavaDoc Reference
Full documentation at [Command JavaDoc](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/autocycle/commands/package-summary.html)
