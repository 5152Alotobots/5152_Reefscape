# Auto Named Commands Utility

A utility class designed to register named commands with PathPlanner's autonomous functionality. This system allows commands to be referenced by name in PathPlanner autonomous routines.

## Purpose
The AutoNamedCommands utility serves as a central registration point for all commands that need to be accessible by name in PathPlanner paths. It maintains a static map of command names to their corresponding Command objects.

## Constructor
This is a utility class with no constructor - all members are static.

## Configuration Required
To use this utility:

1. Add commands to the `commands` HashMap using the pattern:
```java
put("COMMAND_NAME", new YourCommand());
```

2. Call `AutoNamedCommands.setupNamedCommands()` during robot initialization to register all commands with PathPlanner.

## Commands Using This Utility
Any command that needs to be referenced by name in a PathPlanner path must be registered through this utility.

## JavaDoc Reference
Complete documentation can be found [here](PROJECT_ROOT/javadoc/frc/alotobots/package-summary.html)
