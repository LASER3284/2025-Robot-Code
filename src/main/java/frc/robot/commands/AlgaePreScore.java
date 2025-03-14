package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JSConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaePreScore extends SequentialCommandGroup {
    private AlgaeIntake algaeIntake = AlgaeIntake.getInstance();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public AlgaePreScore() {
        addCommands(
            new PivotToAngleEnd(js, rollers, JSConstants.SOURCEINTAKE1)
                .andThen(
                    Commands.parallel( 
                        new ElevatorCommand(ElevatorConstants.ALGAEPRESCORE),  
                        new CarriageCommand(CarriageConstants.ALGAEPRESCORE),
                        new PivotToAngleEnd(js, rollers, JSConstants.ALGAEPRESCORE),
                        new AlgaeDeployEnd(algaeIntake, AlgaeIntakeConstants.ALGAEPRESCORE)
                    ),
                    rollers.algae_roller_on_command(0.8),
                    algaeIntake.rollerSpeed_Command(0.5)
            )
        );
    }
}
