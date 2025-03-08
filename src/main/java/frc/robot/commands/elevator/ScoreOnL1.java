package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral_intake.PivotDeployEnd;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;

public class ScoreOnL1 extends SequentialCommandGroup {
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
    private Pivot pivot = Pivot.getInstance();
    private IntakeRollers irollers = new IntakeRollers();

    public ScoreOnL1() {
        addCommands(
            new SequentialCommandGroup(
                new PivotDeployEnd(pivot, 0.24)
                .andThen(
                    irollers.setMotorSpeed_command(0.0))
                    .andThen(
                        new PivotToAngleEnd(js, rollers, 0.6, 0, 0))
            ))
        ;
    }
}
