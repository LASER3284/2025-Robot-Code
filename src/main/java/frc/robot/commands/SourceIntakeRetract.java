package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.JSConstants;
import frc.robot.commands.defaults.CarriageCommand;
import frc.robot.commands.defaults.ElevatorCommand;
import frc.robot.commands.defaults.PivotDeployEnd;
import frc.robot.commands.defaults.PivotToAngleEnd;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.Pivot;

public class SourceIntakeRetract extends SequentialCommandGroup {
    private Rollers rollers = Rollers.getInstance();
    private JS js = JS.getInstance();
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();
    private Pivot cIntake = Pivot.getInstance();


    public SourceIntakeRetract(double jspose, double speed) {
        addCommands(       
            new PivotDeployEnd(cIntake, .505)   
            .andThen(
                Commands.parallel(
                    new ElevatorCommand(0.2), 
                    new CarriageCommand(CarriageConstants.ALGAEPRESCORE))
                        .until(() -> 
                            elevator.getElevatorPosition() > -.22 && 
                            carriage.getCarriagePosition() > -.5))
            .andThen(
                new PivotToAngleEnd(js, rollers, JSConstants.SOURCEINTAKE1))
            .andThen(
                new PivotDeployEnd(cIntake, .22)),
            rollers.coral_roller_on_command(speed)
        );
    }
}
