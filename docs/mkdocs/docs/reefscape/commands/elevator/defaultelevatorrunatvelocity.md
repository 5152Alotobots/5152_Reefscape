# DefaultElevatorRunAtVelocity Command

## Overview
The `DefaultElevatorRunAtVelocity` command is designed to run the elevator at a specified velocity based on continuous input. This command is typically used as the default command for the elevator subsystem, allowing operator control of elevator movement through a joystick or other input device.

## Required Subsystems
- [Elevator Subsystem](/5152_Reefscape/library/subsystems/elevator)

## Constructor and Parameters

```java
public DefaultElevatorRunAtVelocity(ElevatorSubsystem elevatorSubsystem, DoubleSupplier input)
```

The command requires the following parameters:

- **elevatorSubsystem**: The elevator subsystem instance this command will control
- **input**: A `DoubleSupplier` that provides normalized velocity input values. This supplier typically returns values in the range of -1.0 to 1.0:
    - Positive values move the elevator upward
    - Negative values move the elevator downward
    - Zero stops the elevator

The command scales this input by the maximum speed constant defined in the elevator constants.

## Execution Behavior
- When executed, the command continuously calculates the target velocity by multiplying the input value by the maximum speed
- The command runs until interrupted and never finishes on its own
- When the command ends (due to interruption), it stops the elevator to prevent unwanted movement

## Configuration Requirements
No additional configuration is required. This command uses the `MAX_SPEED` constant from the Elevator Constants configuration to scale the input values appropriately.

## JavaDoc Reference
For detailed API documentation, see the [Elevator Commands JavaDoc](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/elevator/commands/package-summary.html).
