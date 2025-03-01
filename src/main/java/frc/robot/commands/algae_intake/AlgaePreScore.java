package frc.robot.commands.algae_intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaePreScore extends SequentialCommandGroup {
    private AlgaeIntake algaeIntake = new AlgaeIntake();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public AlgaePreScore() {
        new PivotToAngle(js, rollers, 0.6, 0, 0).until(() -> js.isAtSetpoint(0.6));
        new SequentialCommandGroup(
            new AlgaeDeploy(algaeIntake, Inches.of(0)),
            algaeIntake.rollerSpeed_Command(0),
            rollers.algae_roller_on_command(0)
        );
    }
}
