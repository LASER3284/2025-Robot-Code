package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Rollers;

public class ProcessorScore extends SequentialCommandGroup{
    private Rollers rollers = Rollers.getInstance();

    public ProcessorScore() {
        addCommands(
            
            rollers.algae_roller_on_command(-1)
        );
    }
}
