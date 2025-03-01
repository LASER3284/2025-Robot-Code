package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class CoralIntake extends SequentialCommandGroup {
    private IntakeRollers irollers = new IntakeRollers();
    private Rollers rollers = Rollers.getInstance();
    private JS js = JS.getInstance();
    private Pivot pivot = Pivot.getInstance();
    private Carriage carriage = new Carriage();

    public CoralIntake(Elevator elevator, double js_goal, double pivot_goal) {

        addCommands(
            new PivotDeploy(pivot, pivot_goal).until(() -> pivot.isAtSetpoint(pivot_goal)),
            new WaitCommand(0.5),
            irollers.setMotorSpeed_command(0.5),
            new WaitCommand(0.5),
            new PivotToAngle(js, rollers, 0.75, 0.3, 0)
        );
    }

}
