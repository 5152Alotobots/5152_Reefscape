package frc.alotobots.reefscape.subsystems.autocycle.reef.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.alotobots.Constants;
import frc.alotobots.reefscape.subsystems.autocycle.reef.AutoCycleReefSubsystem;
import org.littletonrobotics.junction.Logger;

public class PathfindToSelectedReefBranch extends InstantCommand {
    private AutoCycleReefSubsystem autoCycleReefSubsystem;
    public PathfindToSelectedReefBranch(AutoCycleReefSubsystem autoCycleReefSubsystem) {
        this.autoCycleReefSubsystem = autoCycleReefSubsystem;
    }

    @Override
    public void initialize() {
        String pathName = autoCycleReefSubsystem.getSelectedPathName();
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            Command pathCommand =
                    AutoBuilder.pathfindThenFollowPath(
                            path, Constants.tunerConstants.getPathfindingConstraints());
            pathCommand.schedule(); // Schedule the pathfinding command immediately
        } catch (Exception e) {
            String errorMessage = "Failed to load path: " + pathName;
            Logger.recordOutput("BranchSelection/Error", errorMessage);
            new PrintCommand(errorMessage + " Not following path!").schedule();
        }
    }
}
