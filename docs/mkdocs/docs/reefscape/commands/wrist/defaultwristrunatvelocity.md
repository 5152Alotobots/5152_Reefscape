# DefaultWristRunAtVelocity Command

## Overview
The `DefaultWristRunAtVelocity` command is designed to run the wrist at a specified velocity based on continuous input. This command is typically used as the default command for the wrist subsystem, allowing operator control of wrist movement through a joystick or other input device.

## Required Subsystems
- [Wrist Subsystem](/5152_Reefscape/library/subsystems/wrist)

## Constructor and Parameters

```java
public DefaultWristRunAtVelocity(WristSubsystem wristSubsystem, DoubleSupplier input)
```

The command requires the following parameters:

- **wristSubsystem**: The wrist subsystem instance this command will control
- **input**: A `DoubleSupplier` that provides normalized velocity input values. This supplier typically returns values in the range of -1.0 to 1.0:
    - Positive values move the wrist downwards
    - Negative values move the wrist upwards
    - Zero stops the wrist

The command scales this input by the maximum speed constant defined in the wrist constants.

## Execution Behavior
- When executed, the command continuously calculates the target velocity by multiplying the input value by the maximum speed
- The command runs until interrupted and never finishes on its own
- When the command ends (due to interruption), it stops the wrist to prevent unwanted movement

## Configuration Requirements
No additional configuration is required. This command uses the `MAX_SPEED` constant from the Wrist Constants configuration to scale the input values appropriately.

## JavaDoc Reference
For detailed API documentation, see the [Wrist Commands JavaDoc](/5152_Reefscape/javadoc/frc/alotobots/reefscape/subsystems/wrist/commands/package-summary.html).
