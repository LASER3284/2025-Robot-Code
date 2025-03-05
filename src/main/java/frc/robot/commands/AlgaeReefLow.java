package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaeReefLow extends SequentialCommandGroup {
    private Elevator elevator = new Elevator();
    private Carriage carriage = new Carriage();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
    private final CommandXboxController operator = new CommandXboxController(1);

    public AlgaeReefLow() {
        addCommands(
      
         rollers.algae_roller_on_command(0.6),
             new PivotToAngleEnd(js, rollers, .47 ,0 , 0)
             .andThen(
                Commands.parallel(
                new CarriageCommand(15),
                new ElevatorCommand(-6),
                new PivotToAngle(js, rollers, .47 ,0 , 0)
                )
             )
             

            // new CarriageCommand(car),
            // new ElevatorCommand(ele)

        );
    }
}
