package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class SourceIntake extends SequentialCommandGroup {
    private Rollers rollers = Rollers.getInstance();
    private JS js = JS.getInstance();
    private Carriage carriage = new Carriage();

    public SourceIntake() {
        addCommands(
            new PivotToAngle(js, rollers, 0.9, 0.0, 0)
            .until(() -> js.isAtSetpoint(0.9)),
            new WaitCommand(0.5),
            carriage.carriageCommand(13));
    }
}
