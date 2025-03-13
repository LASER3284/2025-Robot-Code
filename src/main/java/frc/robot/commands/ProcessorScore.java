package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Rollers;

public class ProcessorScore extends SequentialCommandGroup{
    private Rollers rollers = Rollers.getInstance();
    private AlgaeIntake ae = AlgaeIntake.getInstance();

    public ProcessorScore() {
        addCommands(
            
            rollers.algae_roller_on_command(-0.5),
            ae.rollerSpeed_Command(0)
        );
    }
}
