package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class NetScore extends SequentialCommandGroup {
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public NetScore(double js1 ,double car, double ele) {
        addCommands(
            new SequentialCommandGroup(
           new PivotToAngleEnd(js, rollers, 0.5, 0, 0)
            .andThen(
                new ParallelCommandGroup(
                    Commands.parallel(new CarriageCommand(car), new ElevatorCommand(ele)).until(() -> carriage.getCarriagePosition() < -6)
               // new CarriageCommand(car)), new ElevatorCommand(ele))
            )
            .andThen(Commands.parallel(new CarriageCommand(car), new ElevatorCommand(ele), new PivotToAngleEnd(js, rollers, js1, 0.0, 0)).until(() -> elevator.getElevatorPosition() < -18 && carriage.getCarriagePosition() < -18)
            .andThen(Commands.parallel(new CarriageCommand(car), new ElevatorCommand(ele), new PivotToAngleEnd(js, rollers, .7, 0.0, 0)))
            //.andThen(new PivotToAngleEnd(js, rollers, js1, 0.0, 0))
            ))))
        ;
    }
}
