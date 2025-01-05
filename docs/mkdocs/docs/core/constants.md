# Constants

The robot-wide constants class that defines core configuration values and runtime modes. This class serves as the central location for all robot-wide constants and device configurations.

## Purpose
The Constants class provides:
- Runtime mode configuration (REAL/SIM/REPLAY)
- CAN device ID assignments
- Global configuration values
- Hardware device mappings

## Configuration Required
1. Set simMode to the desired simulation mode (SIM/REPLAY) when not running on real hardware
2. Configure CAN IDs for all devices to match physical hardware
3. Update tuner constants for drive system configuration

## Dependencies
- Robot mode affects initialization in [RobotContainer](/5152_Template/core/robotcontainer)
- CAN IDs must match physical hardware configuration
- Tuner constants must match drive system hardware

## JavaDoc Reference
Complete documentation can be found [here](PROJECT_ROOT/javadoc/frc/alotobots/package-summary.html)
