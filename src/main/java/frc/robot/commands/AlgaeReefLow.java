package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaeReefLow extends SequentialCommandGroup {
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public AlgaeReefLow() {
        addCommands(
      
         rollers.algae_roller_on_command(0.35),
             new PivotToAngleEnd(js, rollers, .47 ,0 , 0)
             .andThen(
                Commands.parallel(
                new CarriageCommand(13),
                new ElevatorCommand(0),
                new PivotToAngle(js, rollers, .47 ,0 , 0)
                )
             )
             

            // new CarriageCommand(car),
            // new ElevatorCommand(ele)

        );
    }
}
