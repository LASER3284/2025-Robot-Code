package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.coral_intake.PivotDeployEnd;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngleEnd;
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
            new CarriageCommand(0.5)

        ).until(() -> elevator.getElevatorPosition() > -.22 && carriage.getCarriagePosition() > -.5))
    .andThen(
    new PivotToAngleEnd(js, rollers, .55, 0, 0)
    )
    .andThen(
        new PivotDeployEnd(cIntake, .22)
    ),
    rollers.coral_roller_on_command(speed)


            // new ParallelCommandGroup(
            //     new CarriageCommand(0.2),
            //     new ElevatorCommand(0.2),
            //     cIntake.setGoalPose()
            // ).until(() -> carriage.isAtSetpoint(0.2) && elevator.isAtHome(0.2) && cIntake.isAtSetpoint(0.01)),
            // new ParallelCommandGroup(
            //     new PrintCommand("is this working"),
            // new PivotToAngle(js, rollers, jspose, 0.3, 0)
            // .until(() -> js.isAtSetpoint(jspose))),
            // rollers.coral_roller_on_command(speed));
        );
    }
}
