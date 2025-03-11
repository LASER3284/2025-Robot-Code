package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.JSConstants;
import frc.robot.commands.defaults.AlgaeDeployEnd;
import frc.robot.commands.defaults.CarriageCommand;
import frc.robot.commands.defaults.ElevatorCommand;
import frc.robot.commands.defaults.PivotToAngleEnd;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ProcessorPreScore extends SequentialCommandGroup{
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
    private AlgaeIntake algaeIntake = AlgaeIntake.getInstance();

    public ProcessorPreScore() {
        addCommands(
            algaeIntake.rollerSpeed_Command(0),
            rollers.algae_roller_on_command(0),
            Commands.parallel(
                new AlgaeDeployEnd(algaeIntake, Inches.of(-9)),
                new PivotToAngleEnd(js, rollers, JSConstants.PROCESSORPRESCORE) 
            )
            .andThen(  
                Commands.parallel(
                    new CarriageCommand(CarriageConstants.PROCESSORPRESCORE), 
                    new ElevatorCommand(0.4))
            )
        );
    }
}
