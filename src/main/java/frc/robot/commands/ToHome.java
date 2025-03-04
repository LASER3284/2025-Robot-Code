package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;
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
            new PivotToAngleEnd(js, rollers, 0.5, 0, 0)
             .andThen(
           new ParallelCommandGroup(
                    Commands.parallel(new PivotToAngleEnd(js, rollers, 0.5, 0, 0), 
                    new CarriageCommand(.5),
                    new ElevatorCommand(.5))
      
         ) 
            )
        );
        
    }
}
