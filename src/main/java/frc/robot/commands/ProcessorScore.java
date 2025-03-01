package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ProcessorScore extends SequentialCommandGroup{
    private Elevator elevator = new Elevator();
    private Carriage carriage = new Carriage();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
    private AlgaeIntake algaeIntake = new AlgaeIntake();

    public ProcessorScore() {
        addCommands(
            new ParallelCommandGroup(
                carriage.carriageCommand(0.1),
                elevator.elevatorCommand(0.1)
            ).until(() -> carriage.isAtSetpoint(0.1) && elevator.isAtHome(0.1)),
            new PivotToAngleEnd(js, rollers, 0.4, 0, 0)

        );
    }
}
