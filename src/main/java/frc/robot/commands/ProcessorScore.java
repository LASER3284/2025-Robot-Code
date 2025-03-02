package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ProcessorScore extends SequentialCommandGroup{
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public ProcessorScore() {
        addCommands(
            new SequentialCommandGroup(
                new PivotToAngle(js, rollers, 0.5, 0, 0)
            ),
            rollers.algae_roller_on_command(-0.8)
        );
    }
}
