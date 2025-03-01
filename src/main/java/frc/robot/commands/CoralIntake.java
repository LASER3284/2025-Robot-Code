package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.coral_intake.PivotDeployEnd;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;
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
    private Elevator elevator = new Elevator();

    public CoralIntake(double js_goal, double pivot_goal) {

        addCommands(
          new PivotDeployEnd(pivot, pivot_goal).until(() -> pivot.isAtSetpoint(0.3)),
            new SequentialCommandGroup(
            new PivotDeployEnd(pivot, pivot_goal).until(() -> pivot.isAtSetpoint(pivot_goal)),
            new PivotToAngleEnd(js, rollers, 0.95, 0.3, 0).until(() -> js.isAtSetpoint(0.95)),
            rollers.coral_roller_on_command(0.8)),
            irollers.setMotorSpeed_command(0.65),
            elevator.elevatorCommand(3).until(() -> elevator.isAtHome(3)));
            // new SequentialCommandGroup(
            //     new PrintCommand("working"),
            //     elevator.elevatorCommand(3));
    }
}
