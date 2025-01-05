# RobotContainer Class

The central class that manages all subsystems and commands for the robot. It handles subsystem initialization, command configuration, and autonomous routine selection.

## Purpose
RobotContainer serves as the main organizational structure for the robot's subsystems and commands. It:
- Initializes all subsystems with appropriate IO implementations for real hardware, simulation, or replay modes
- Configures subsystem default commands
- Sets up command bindings
- Manages autonomous routine selection
- Provides system identification routines for tuning

## Subsystems Managed
The RobotContainer manages all robot subsystems, handling their initialization, dependencies, and interactions. Each subsystem is initialized with the appropriate IO layer based on whether the robot is running in real hardware, simulation, or replay mode.

## Configuration Required
1. Constants.currentMode must be set appropriately for target environment (REAL/SIM/REPLAY)
2. PathPlanner autonomous routines must be properly configured
3. Hardware devices must be configured with correct CAN IDs (see [Constants](/5152_Template/core/constants))
4. Camera configurations must be set in vision constants

## JavaDoc Reference
Complete documentation can be found [here](PROJECT_ROOT/javadoc/frc/alotobots/package-summary.html)
