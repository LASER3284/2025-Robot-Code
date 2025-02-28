package frc.robot.commands;

import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.pivot.PivotToAngle;

public class PreScore extends SequentialCommandGroup {
    private Carriage carriage = new Carriage();
    private JS js;
    private Rollers rollers;
    

    public PreScore() {
        js = new JS(rollers);

        addCommands(
            new PivotToAngle(js, rollers, 0.5, 0).until(() -> js.isAtSetpoint(0.5)),
            carriage.carriageCommand(6),
            new WaitCommand(0.5),
            new PivotToAngle(js, rollers, 0.4, 0.0)
        );

    }
}
