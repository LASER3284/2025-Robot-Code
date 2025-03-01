package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ScoreOnReef extends SequentialCommandGroup {
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public ScoreOnReef(double js1 ,double car, double ele) {
        addCommands(
            new PivotToAngle(js, rollers, 0.5, 0, 0).until(() -> js.isAtSetpoint(0.5)),
            new WaitCommand(0.5),
            carriage.carriageCommand(car),
            elevator.elevatorCommand(ele),
            new PivotToAngle(js, rollers, js1, 0.0, 0)
        );
    }
}
