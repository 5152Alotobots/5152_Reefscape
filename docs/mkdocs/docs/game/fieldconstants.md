# Field Constants Subsystem

The Field Constants subsystem provides field-specific positions and measurements for the Reef game field. It handles the storage and retrieval of positions for reef branches, algae positions, and game piece floor positions for both red and blue alliances.

## Constructor and Values

This is a utility class with no constructor. It uses static initialization to set up the following key components:

- **Reef Branch Positions**: Stores positions for branches A-L at three different height levels (L2, L3, L4)
- **Reef Algae Positions**: Stores positions for algae between specific branch pairs (AB, CD, EF, GH, IJ, KL)
- **Floor Positions**: Stores positions for both coral and algae game pieces on the field floor

## Commands Using This Subsystem

The Field Constants subsystem is primarily used as a reference by other subsystems and commands for:

- Path planning to specific field locations
- Auto-targeting systems
- Vision system pose estimation
- Autonomous routines

## Configuration Requirements

The following need to be configured correctly:

1. Field measurements must be verified and updated if field specifications change
2. Alliance color must be set correctly in robot code to ensure proper pose mirroring
3. All positions are measured from the blue alliance perspective and automatically mirrored for red alliance

## Reference Documentation

For detailed technical documentation, see the [JavaDoc Reference](/5152_Template/javadoc/frc/alotobots/reefscape/package-summary.html)