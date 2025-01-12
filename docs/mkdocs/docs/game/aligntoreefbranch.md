# AlignToReefBranch Command

The AlignToReefBranch command is responsible for precisely positioning the robot relative to a specific reef branch on the field. It uses PathPlanner's holonomic drive controller to maintain proper orientation and position throughout the alignment process, taking into account the current alliance color.

## Required Subsystems
- [SwerveDrive Subsystem](/5152_Reefscape/library/subsystems/swerve)

## Constructor Parameters
- `swerveDrive`: The SwerveDrive subsystem instance used for robot movement
- `targetBranch`: The ReefBranch enumeration specifying which branch to align with
- `targetLevel`: The Level enumeration specifying the vertical level of the target branch

## Configuration Requirements
The command relies on several constants that must be properly configured:
- `ALIGNMENT_RADIUS`: The maximum distance from the target at which alignment will be attempted
- `POSITION_TOLERANCE`: The minimum distance from target considered "aligned"
- `ROBOT_LENGTH`: The physical length of the robot used for positioning calculations
- Proper configuration of the PathPlanner holonomic drive controller parameters

## Technical Details
- Uses PathPlanner's PPHolonomicDriveController for precise movement control
- Automatically adjusts target positions based on alliance color
- Maintains proper robot orientation throughout alignment
- Includes built-in position tolerance and alignment radius checks
- Provides extensive logging through AdvantageKit

## Javadoc Reference
- [Command Documentation](5152_Reefscape/javadoc/frc/alotobots/reefscape/commands/scoring/reef/alignment/package-summary.html)
