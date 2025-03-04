package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.coral_intake.PivotDeploy;
import frc.robot.commands.coral_intake.PivotDeployEnd;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.commands.pivot.PivotToAngleEnd;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;

public class CoralIntake extends SequentialCommandGroup {
    private IntakeRollers irollers = new IntakeRollers();
    private Rollers rollers = Rollers.getInstance();
    private JS js = JS.getInstance();
    private Pivot pivot = Pivot.getInstance();
    private Carriage carriage = new Carriage();
    private Elevator elevator = new Elevator();

    public CoralIntake() {
        addCommands(
           
        //  new PivotDeployEnd(pivot, .47).until(()-> pivot.getPivotPosition() > 0.36).andThen(
          new PivotToAngleEnd(js, rollers, .6, 0, 0)
          .andThen(
        Commands.parallel(
          new ElevatorCommand(0.2), 
          new CarriageCommand(0.5)
        )
        ).until(() -> elevator.getElevatorPosition() > -.22 && carriage.getCarriagePosition() > -.5)
        .andThen(
              new PivotDeployEnd(pivot, .505)   
        )
            .andThen(new PivotToAngleEnd(js, rollers, 0.945, 0.0, .00),
            irollers.setMotorSpeed_command(0.8),
            rollers.coral_roller_on_command(0.9))
            );       
    }
}
