package frc.robot.commands;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral_intake.PivotDeployEnd;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;

public class PreScore extends SequentialCommandGroup {
    private Carriage carriage = new Carriage();
    private Rollers rollers = Rollers.getInstance();
    private Pivot pivot = Pivot.getInstance();
    private IntakeRollers irollers = new IntakeRollers();
    private JS js = JS.getInstance();
    private Elevator elevator = new Elevator();

    public PreScore() {
        
        addCommands(
            // new CarriageCommand(.1).until(() -> carriage.isAtSetpoint(.1) ),
            // new ElevatorCommand(0.1).until(() -> elevator.isAtHome(0.1)),
            // irollers.setMotorSpeed_command(0),
            // rollers.coral_roller_on_command(0),
            // new PivotToAngle(js, rollers, 0.52, 0, 0).until(() -> js.isAtSetpoint(0.5)),
            // new PivotDeployEnd(pivot, 0.03).until(() -> pivot.isAtSetpoint(0.03)),
            // new CarriageCommand(6).until(() -> carriage.isAtSetpoint(6))
            
            new SequentialCommandGroup(
                irollers.setMotorSpeed_command(0),
                rollers.coral_roller_on_command(0),
                new ParallelCommandGroup(new CarriageCommand(0)) //new ElevatorCommand(0))
                .andThen(new PivotToAngleEnd(js, rollers, 0.52, 0.0, .00))
                .andThen(new CarriageCommand(6))


            )
        );

    }
}
