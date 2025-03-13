package frc.alotobots.reefscape.subsystems.autocycle.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.alotobots.library.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.alotobots.library.subsystems.swervedrive.commands.DrivePrecisionAlign;
import frc.alotobots.reefscape.subsystems.autocycle.AutoCycleSubsystem;

public class PrecisionAlignToReef extends Command {

    private final AutoCycleSubsystem autoCycleSubsystem;
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final DrivePrecisionAlign drivePrecisionAlign;
    public PrecisionAlignToReef(AutoCycleSubsystem autoCycleSubsystem, SwerveDriveSubsystem swerveDriveSubsystem) {
        this.autoCycleSubsystem = autoCycleSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.drivePrecisionAlign = new DrivePrecisionAlign(swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        autoCycleSubsystem.getPathPlannerManager()
                .getPathEndPose(autoCycleSubsystem.getState().getSelectedReefBranchPathName())
                .ifPresent(endPose -> drivePrecisionAlign.applyRequest(() -> endPose));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return drivePrecisionAlign.isFinished();
    }
}
