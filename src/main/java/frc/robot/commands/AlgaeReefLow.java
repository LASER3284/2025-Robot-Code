package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JSConstants;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaeReefLow extends SequentialCommandGroup {
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public AlgaeReefLow() {
        addCommands(
            rollers.algae_roller_on_command(0.35),
                new PivotToAngleEnd(js, rollers, JSConstants.ALGAEREEFLOW)
                    .andThen(
                        Commands.parallel(
                            new CarriageCommand(JSConstants.ALGAEREEFLOW),
                            new ElevatorCommand(ElevatorConstants.ZERO),
                            new PivotToAngle(js, rollers, JSConstants.ALGAEREEFLOW)
                        )
                    )
        );
    }
}
