package frc.robot.commands.algae_intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaePreScore extends SequentialCommandGroup {
    private AlgaeIntake algaeIntake = AlgaeIntake.getInstance();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public AlgaePreScore() {
        addCommands(

        new PivotToAngleEnd(js, rollers, 0.6, 0, 0)
        .andThen(
            Commands.parallel( 
                new ElevatorCommand(0.2),  
                new CarriageCommand(0.5),
                new PivotToAngleEnd(js, rollers, 0.47, 0, 0),
                new AlgaeDeployEnd(algaeIntake,Inches.of(-34))
            ),
    
            rollers.algae_roller_on_command(0.8),
            algaeIntake.rollerSpeed_Command(0.5)
            
           
           )

         
           
            
            
        );


        //,
        // new PivotToAngle(js, rollers, 0.47, 0, 0).until(() -> js.isAtSetpoint(0.45))
        // .andThen(new SequentialCommandGroup(
        //     new AlgaeDeploy(algaeIntake, Inches.of(-34)).until(() -> algaeIntake.isAtSetpoint(-34))))
        // .andThen(rollers.algae_roller_on_command(0.8)));
        
    }
}
