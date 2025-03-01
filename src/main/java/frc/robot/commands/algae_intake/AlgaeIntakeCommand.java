package frc.robot.commands.algae_intake;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaeIntakeCommand extends SequentialCommandGroup{
    private AlgaeIntake algaeintake;
    private Distance distance;
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public AlgaeIntakeCommand() {
        addCommands(
            new ParallelCommandGroup(
                new AlgaeDeploy(algaeintake, distance),
                algaeintake.rollerSpeed_Command(-0.5),
                rollers.algae_roller_on_command(0.5)
            ),
            new PivotToAngle(js, rollers, 0.9, 0, 0.3)
        );
    }
}
