package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JSConstants;
import frc.robot.commands.defaults.CarriageCommand;
import frc.robot.commands.defaults.ElevatorCommand;
import frc.robot.commands.defaults.PivotToAngleEnd;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaeReefHigh extends SequentialCommandGroup {
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
 

    public AlgaeReefHigh() {
        addCommands(
            rollers.algae_roller_on_command(0.35),
                new PivotToAngleEnd(js, rollers, JSConstants.ALGAEREEFHIGH)
                    .andThen(
                        Commands.parallel(
                            new CarriageCommand(JSConstants.ALGAEREEFHIGH),
                            new ElevatorCommand(ElevatorConstants.ALGAEREEFHIGH)
                                .andThen(
                                    new PivotToAngleEnd(js, rollers, JSConstants.ALGAEREEFHIGH)
                                )
                        )           
                    )
        );
    }
}
