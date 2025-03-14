package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class ScoreOnReef extends SequentialCommandGroup {
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public ScoreOnReef(double jsa ,double car, double ele) {
        addCommands(
            new SequentialCommandGroup(
                new PivotToAngleEnd(js, rollers, 0.5)
                    .andThen(
                        new ParallelCommandGroup(
                            Commands.parallel(
                                new CarriageCommand(car), 
                                new ElevatorCommand(ele))
                                    .until(() -> carriage.getCarriagePosition() < -6)
                        )
                    .andThen(
                        Commands.parallel(
                            new CarriageCommand(car), 
                            new ElevatorCommand(ele),
                            new PivotToAngleEnd(js, rollers, jsa)
                        )
                    )
                    )
            )
        );
    }
}
