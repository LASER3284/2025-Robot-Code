package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class L2 extends SequentialCommandGroup {
    private Carriage carriage;
    private JS js;
    private Rollers rollers;

    public L2() {
        addCommands(
            new PivotToAngle(js, rollers, 0.5, 0).until(() -> js.isAtSetpoint(0.5))
            // carriage.carriageCommand(15),
            // new WaitCommand(0.5),
            // new PivotToAngle(0.4, 0.0)
        );

    }
}
