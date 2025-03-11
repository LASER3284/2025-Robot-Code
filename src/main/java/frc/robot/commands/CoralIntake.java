package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.JSConstants;
import frc.robot.commands.defaults.PivotDeployEnd;
import frc.robot.commands.defaults.PivotToAngleEnd;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;

public class CoralIntake extends SequentialCommandGroup {
    private IntakeRollers irollers = new IntakeRollers();
    private Rollers rollers = Rollers.getInstance();
    private JS js = JS.getInstance();
    private Pivot pivot = Pivot.getInstance();

    public CoralIntake() {
        addCommands(
            new PivotToAngleEnd(js, rollers, JSConstants.CORALINTAKE)
                .andThen(
                    new PivotDeployEnd(pivot, .505)   
                ).andThen(
                    irollers.setMotorSpeed_command(0.5)
                )   
        );  
    }
}
