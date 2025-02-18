package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;

public class SourceIntake extends SequentialCommandGroup {
    private JS js = new JS();
    public Rollers rollers = new Rollers(); 

    public SourceIntake(JS js, Rollers rollers) {
        this.js = js;
        this.rollers = rollers;

        // addCommands(new PivotToAngle(js, rollers, Degrees.of(0.06))
        //         .andThen(rollers.coral_roller_on_command(0.5)));
    }
}
