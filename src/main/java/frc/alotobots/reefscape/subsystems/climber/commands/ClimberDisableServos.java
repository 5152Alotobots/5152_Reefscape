package frc.alotobots.reefscape.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.alotobots.reefscape.subsystems.climber.ClimberSubsystem;

public class ClimberDisableServos extends InstantCommand {
    private final ClimberSubsystem climberSubsystem;
    public ClimberDisableServos(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        climberSubsystem.disableServos();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
