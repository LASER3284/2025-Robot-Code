package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.algae_intake.AlgaeDeploy;
import frc.robot.commands.algae_intake.AlgaeDeployEnd;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ProcessorPreScore extends SequentialCommandGroup{
    private Elevator elevator = new Elevator();
    private Carriage carriage = new Carriage();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
    private AlgaeIntake algaeIntake = AlgaeIntake.getInstance();

    public ProcessorPreScore() {
        addCommands(
            algaeIntake.rollerSpeed_Command(0),
            rollers.algae_roller_on_command(0),
            Commands.parallel(
                new AlgaeDeployEnd(algaeIntake, Inches.of(-9)),
                new PivotToAngleEnd(js, rollers, 0.5, 0, 0) 
                )
            
            .andThen(  
                Commands.parallel(new CarriageCommand(0.4), new ElevatorCommand(0.4))
                
            )

        );
    }
}
