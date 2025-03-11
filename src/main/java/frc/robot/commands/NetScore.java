package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.JSConstants;
import frc.robot.commands.defaults.CarriageCommand;
import frc.robot.commands.defaults.ElevatorCommand;
import frc.robot.commands.defaults.PivotToAngleEnd;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class NetScore extends SequentialCommandGroup {
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();

    public NetScore() {
        addCommands(
            new SequentialCommandGroup(
                new PivotToAngleEnd(js, rollers, JSConstants.TOHOME)
                    .andThen(
                        new ParallelCommandGroup(
                            Commands.parallel(
                                new CarriageCommand(CarriageConstants.NETSCORE), 
                                new ElevatorCommand(ElevatorConstants.NETSCORE))
                                    .until(() -> carriage.getCarriagePosition() < -6)
            )
            .andThen(
                Commands.parallel(
                    new CarriageCommand(CarriageConstants.NETSCORE), 
                    new ElevatorCommand(ElevatorConstants.NETSCORE), 
                    new PivotToAngleEnd(js, rollers, JSConstants.SCOREONL1))
                        .until(() -> elevator.getElevatorPosition() < -18 
                            && carriage.getCarriagePosition() < -18)
            .andThen(
                Commands.parallel(
                    new CarriageCommand(CarriageConstants.NETSCORE), 
                    new ElevatorCommand(ElevatorConstants.NETSCORE), 
                    new PivotToAngleEnd(js, rollers, JSConstants.NETSCORE)))
            ))))
        ;
    }
}
