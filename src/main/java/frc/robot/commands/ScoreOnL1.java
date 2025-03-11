package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.JSConstants;
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
                        new PivotToAngleEnd(js, rollers, JSConstants.SCOREONL1))
            ))
        ;
    }
}
