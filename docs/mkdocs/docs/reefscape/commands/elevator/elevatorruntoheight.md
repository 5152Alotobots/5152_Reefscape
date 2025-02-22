# ElevatorRunToHeight Command

A command that moves the elevator to a specified target height using closed-loop position control. The command can either end once the target is reached or continuously maintain the position.

## Required Subsystems
- [Elevator Subsystem](/5152_Reefscape/reefscape/subsystems/elevator)

## Constructor Parameters
```java
public ElevatorRunToHeight(ElevatorSubsystem elevatorSubsystem, Distance targetHeight, boolean holdPosition)
```
- `elevatorSubsystem`: The elevator subsystem instance to control
- `targetHeight`: The desired height for the elevator to reach (automatically clamped between MIN_HEIGHT and MAX_HEIGHT)
- `holdPosition`: Optional parameter to maintain position after reaching target

## Configuration Requirements
1. PID Configuration
    - Position mode PID gains must be properly tuned
    - AT_SET_POINT_THRESHOLD must be set to an appropriate value

2. Hardware Limits
    - MAX_HEIGHT and MIN_HEIGHT must be configured in ElevatorConstants
    - Soft limits should be enabled to prevent mechanical damage

3. Motor Configuration
    - Brake mode should be enabled for position holding
    - Feedback sensor must be properly configured and zeroed

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/commands/package-summary.html)
