# Robot Class

The main robot class that handles the robot's lifecycle and operating modes. This class extends LoggedRobot to integrate with AdvantageKit logging framework and manages all robot operations.

## Purpose
The Robot class serves as the main entry point for the robot code and handles:
- Mode transitions (autonomous, teleop, test, disabled)
- Command scheduling
- AdvantageKit logging setup
- High-priority periodic updates

## Constructor Parameters
The Robot constructor initializes several key components:
- Sets up AdvantageKit logging based on the current mode (REAL/SIM/REPLAY)
- Records build metadata (Git info, build date, etc.)
- Creates the RobotContainer instance

## Requirements

### Configuration
1. USB drive mounted at "/U/logs" for logging in REAL mode
2. Properly configured Constants.currentMode
3. Valid RobotContainer implementation

### Dependencies
- [RobotContainer](/5152_Template/core/robotcontainer)
- [Constants](/5152_Template/core/constants)
- AdvantageKit logging framework

## JavaDoc Reference
Complete documentation can be found [here](PROJECT_ROOT/javadoc/frc/alotobots/package-summary.html)
