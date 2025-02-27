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

## Configuration
- Ensure PID constants are properly tuned
- Configure MAX_HEIGHT and MIN_HEIGHT in ElevatorConstants
- AT_TARGET_HEIGHT_POSITION_THRESHOLD must be set appropriately for position tolerance
- Soft limits should be enabled to prevent mechanical damage
- The command will use the appropriate PID slot based on the current game element
- Brake mode should be properly configured in the motor controller for position holding

2. Hardware Limits
    - MAX_HEIGHT and MIN_HEIGHT must be configured in ElevatorConstants
    - Soft limits should be enabled to prevent mechanical damage

3. Motor Configuration
    - Brake mode should be enabled for position holding
    - Feedback sensor must be properly configured and zeroed

[View Javadoc Reference](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/commands/package-summary.html)
