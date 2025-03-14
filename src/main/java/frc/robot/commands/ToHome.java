package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.JSConstants;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ToHome extends SequentialCommandGroup{
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public ToHome() {
        addCommands(
            rollers.algae_roller_on_command(0),
            new PivotToAngleEnd(js, rollers, JSConstants.TOHOME)
                .andThen(
                    new ParallelCommandGroup(
                        Commands.parallel(
                            new PivotToAngleEnd(js, rollers, JSConstants.TOHOME), 
                            new CarriageCommand(CarriageConstants.TOHOME),
                            new ElevatorCommand(.5))
                    ) 
                )
        );
    }
}
