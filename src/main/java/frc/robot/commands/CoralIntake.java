package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;

public class CoralIntake extends SequentialCommandGroup {
    private JS js = new JS();
    private IntakeRollers irollers = new IntakeRollers();
    private Rollers rollers = new Rollers();
    private Pivot pivot = new Pivot();

    public CoralIntake(JS js, Rollers rollers ,IntakeRollers irollers, Pivot pivot, double js_goal, double pivot_goal) {
        this.js = js;
        this.rollers = rollers;
        this.irollers = irollers;
        this.pivot = pivot;

        addCommands(new PivotToAngle(js, rollers, Degrees.of(js_goal), 0)
            .andThen(new PivotDeploy(pivot, Degrees.of(pivot_goal))));
    }

}
