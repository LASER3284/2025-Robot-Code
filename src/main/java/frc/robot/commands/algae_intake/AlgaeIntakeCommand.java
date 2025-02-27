package frc.robot.commands.algae_intake;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeIntakeCommand extends SequentialCommandGroup{
    private AlgaeIntake algaeintake;
    private Distance distance;

    public AlgaeIntakeCommand() {
        addCommands(
            new AlgaeDeploy(algaeintake, distance),
            algaeintake.rollerSpeed_Command(-0.5)
        );
    }
}
