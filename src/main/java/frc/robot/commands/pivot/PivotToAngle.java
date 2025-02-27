package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.*;
 
public class PivotToAngle extends Command {

    private TrapezoidProfile trappy;
    private JS Pivot;
    double setPoint;
    private double angle;
    double speed;


    public void initialize() {
        Pivot.setGoal(angle);
        Pivot.setSetpoint(new TrapezoidProfile.State(Pivot.getPivotPosition(), 0.0));
    }

    public PivotToAngle(JS js, Rollers rollers, double angle, double speed) {
        Pivot = new JS(rollers);
        this.angle = angle;
        this.speed = speed;

        addRequirements(Pivot);

    }

    public void execute() {
        //rollers.coral_roller_on_command(speed);
        Pivot.setPower(0);
        trappy = new TrapezoidProfile(
            Pivot.getConstraints());

        Pivot.setLastGoal(angle);
        double pose = Pivot.getPivotPosition();
        TrapezoidProfile.State next = trappy.calculate(0.02, Pivot.getSetpoint(), Pivot.getGoal());
        Pivot.setSetpoint(next);
        Pivot.getPID().setSetpoint(next.position);
        double power = Pivot.getPID().calculate(pose);
        SmartDashboard.putNumber("power", power);
        double PIDFFpower = power;
        SmartDashboard.putNumber("PIDFFpower", PIDFFpower);
        Pivot.setPower(-PIDFFpower);
    }

    public void end(boolean interrupted) {
        Pivot.setPower(0);
    }
}

