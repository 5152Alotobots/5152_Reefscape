# Button Bindings Overview

## Driver Controller (Port 0)
Xbox Controller mapping for primary robot control.

### Joysticks
- **Left Stick**
    - Y-Axis: Drive forward/backward
    - X-Axis: Drive left/right
- **Right Stick**
    - X-Axis: Robot rotation

### Triggers
- **Left Trigger**: Turtle (slow) mode
- **Right Trigger**: Turbo (fast) mode

### Buttons
- **A Button**: Toggle drive facing best object
- **B Button**: Hold to pathfind to best object

## Controller Port Assignments
- Driver Controller: Port 0

## Configuration Notes
- Inputs use a deadband of 0.1
- All drive inputs are field-relative
- Speed modifiers affect all drive axes simultaneously
- Button bindings are configured in RobotContainer's configureLogicCommands()
- Raw controller access available through OI class methods

## Related Documentation
- [Operator Interface](/5152_Template/core/oi)
- [Controls Constants](/5152_Template/core/constants)
