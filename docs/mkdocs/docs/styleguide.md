# Code Style Guide

## Project Structure

### Base Package
```
frc.alotobots/
├── Main.java                 # Entry point
├── Robot.java               # Robot lifecycle
├── RobotContainer.java      # Subsystem and command management
├── Constants.java           # Robot-wide constants
└── OI.java                  # Operator interface
```

### Subsystems Structure (Library)
```
library.subsystems/
├── subsystem_name/
│   ├── commands/            # Commands specific to this subsystem
│   ├── constants/          # Subsystem-specific constants
│   ├── io/                # Hardware abstraction layer
│   └── util/              # Utility classes for this subsystem
```

### Subsystems Structure (Game Specific)
```
game.subsystems/
├── subsystem_name/
│   ├── commands/            # Commands specific to this subsystem
│   ├── constants/          # Subsystem-specific constants
│   ├── io/                # Hardware abstraction layer
│   └── util/              # Utility classes for this subsystem
```

All other commands that require multiple subsystems are placed in a subdirectory of game or library named `commands`.

## Naming Conventions

### Files
- Subsystems: `*Subsystem.java` (e.g., `SwerveDriveSubsystem.java`)
- Commands: `*.java` (e.g., `DefaultDrive.java`)
- Constants: `*Constants.java` (e.g., `SwerveDriveConstants.java`)
- IO Interfaces: `*IO.java` (e.g., `ModuleIO.java`)
- IO Implementations: `*IO{Implementation}.java` (e.g., `ModuleIOTalonFX.java`)

### Variables
- Constants: `UPPER_SNAKE_CASE`
- Instance variables: `camelCase`
- Static variables: `camelCase`

## Code Organization

### Subsystem Pattern
Each subsystem should follow this organization:
```java
public class ExampleSubsystem extends SubsystemBase {
    // Hardware/IO variables
    private final ExampleIO io;

    // State variables
    private final ExampleInputsAutoLogged inputs = new ExampleInputsAutoLogged();

    // Constructor
    public ExampleSubsystem(ExampleIO io) {
        this.io = io;
    }

    // Periodic methods
    @Override
    public void periodic() { }

    // Public methods

    // Private helper methods
}
```

### Command Pattern
Commands should follow this organization:
```java
public class ExampleCommand extends CommandBase {
    // Subsystem dependencies
    private final ExampleSubsystem subsystem;

    // Command state

    // Constructor
    public ExampleCommand(ExampleSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    // Command lifecycle methods
    @Override
    public void initialize() { }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() { }
}
```

## Hardware Abstraction
- All hardware interactions must go through IO interfaces
- Real hardware implementations in `io` package
- Simulation implementations in same package with `Sim` suffix
- Use empty implementations for replay mode

## Constants Organization
- Robot-wide constants in `Constants.java`
- Subsystem-specific constants in respective `constants` package
- Use inner classes to organize related constants
- All constants should be final

## Testing and Simulation
- Simulation support implemented for critical subsystems only
    - Required for drive system
    - Optional for other subsystems unless specifically needed
- Characterization commands available for drive system tuning
- Log relevant data using AdvantageKit
- Simulation implementations should be maintained when provided by WPILib or vendor libraries

## Documentation
- JavaDoc required for all public methods
- Class-level documentation explaining purpose
- Constants must include units in documentation
- Command requirements and effects must be documented

## Best Practices
1. Use dependency injection for hardware IO
2. Keep subsystems focused and single-purpose
3. Commands should be small and composable
4. Use AdvantageKit logging consistently
5. Follow WPILib command-based paradigms
6. Maintain simulation support for all features
7. Use PhotonVision conventions for vision processing
8. Implement both real and simulated hardware interfaces

## Version Control
- Feature branches for new development
- Pull requests required for merging
- Continuous Integration checks for style
- Version tags for competition code
- Automated builds for releases
