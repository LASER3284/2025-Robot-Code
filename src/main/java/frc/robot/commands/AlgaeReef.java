package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.CarriageCommand;
import frc.robot.commands.elevator.ElevatorCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class AlgaeReef extends SequentialCommandGroup {
    private Elevator elevator = new Elevator();
    private Carriage carriage = new Carriage();
    private JS js = JS.getInstance();
    private Rollers rollers = Rollers.getInstance();
    private AlgaeIntake algaeIntake = new AlgaeIntake();

    public AlgaeReef(double aspeed, double jsangle, double car, double ele) {
        addCommands(
            algaeIntake.rollerSpeed_Command(aspeed),
            new PivotToAngle(js, rollers, jsangle ,0 , 0).until(() -> js.isAtSetpoint(jsangle)),
            new CarriageCommand(car),
            new ElevatorCommand(ele)

        );
    }
}
