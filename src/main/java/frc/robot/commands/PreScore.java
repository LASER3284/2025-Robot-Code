package frc.robot.commands;

// import frc.robot.subsystems.Carriage;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
//import frc.robot.subsystems.pivotintake.Pivot;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.JSConstants;

public class PreScore extends SequentialCommandGroup {
    private Rollers rollers = Rollers.getInstance();
    private IntakeRollers irollers = IntakeRollers.getInstance();
    private JS js = JS.getInstance();

    public PreScore() {
        addCommands(
            new SequentialCommandGroup(
                irollers.setMotorSpeed_command(0),
                rollers.coral_roller_on_command(0),
                    new ParallelCommandGroup(
                        new CarriageCommand(0)
                    )
                .andThen(
                    new PivotToAngleEnd(js, rollers, JSConstants.PRESCORE))
                .andThen(
                    new CarriageCommand(CarriageConstants.PRESCORE))
            )
        );

    }
}
