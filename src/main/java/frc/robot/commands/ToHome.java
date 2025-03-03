package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ToHome extends SequentialCommandGroup{
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public ToHome() {
        addCommands(
            rollers.algae_roller_on_command(0),
            new PivotToAngle(js, rollers, 0.5, 0, 0).until(() -> js.isAtSetpoint(0.5)),
         //   new WaitCommand(3),
            carriage.carriageCommand(.5),
            elevator.elevatorCommand(.5)
      //      new PivotToAngle(js, rollers, 0.5, 0.0)
        );
    }
}
