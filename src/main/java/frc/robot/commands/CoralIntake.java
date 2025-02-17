package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;

public class CoralIntake extends SequentialCommandGroup {
    private JS js;
    private IntakeRollers irollers;
    private Pivot pivot;

    public CoralIntake(JS js, IntakeRollers irollers, Pivot pivot, double js_goal, double pivot_goal) {
        this.js = js;
        this.irollers = irollers;
        this.pivot = pivot;

        addCommands(new PivotToAngle(js, Degrees.of(js_goal))
            .andThen(new PivotDeploy(pivot, irollers, Degrees.of(pivot_goal))));
    }

}
