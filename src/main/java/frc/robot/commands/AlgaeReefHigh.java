package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaeReefHigh extends SequentialCommandGroup {
    private Elevator elevator = new Elevator();
    private Carriage carriage = new Carriage();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
 

    public AlgaeReefHigh() {
        addCommands(
      
         rollers.algae_roller_on_command(0.6),
             new PivotToAngleEnd(js, rollers, .47 ,0 , 0)
             .andThen(
                new CarriageCommand(13.5),
                new ElevatorCommand(-9),
                new PivotToAngleEnd(js, rollers, .47 ,0 , 0)
             )

            // new CarriageCommand(car),
            // new ElevatorCommand(ele)

        );
    }
}
