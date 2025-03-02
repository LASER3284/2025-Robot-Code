package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.Pivot;

public class SourceIntake extends SequentialCommandGroup {
    private Rollers rollers = Rollers.getInstance();
    private JS js = JS.getInstance();
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();
    private Pivot cIntake = Pivot.getInstance();

    public SourceIntake(double jspose, double speed) {
        addCommands(
            new ParallelCommandGroup(
                new CarriageCommand(0.2),
                new ElevatorCommand(0.2),
                cIntake.setGoalPose()
            ).until(() -> carriage.isAtSetpoint(0.2) && elevator.isAtHome(0.2) && cIntake.isAtSetpoint(0.01)),
            new ParallelCommandGroup(
                new PrintCommand("is this working"),
            new PivotToAngle(js, rollers, jspose, 0.3, 0)
            .until(() -> js.isAtSetpoint(jspose))),
            rollers.coral_roller_on_command(speed));
    }
}
