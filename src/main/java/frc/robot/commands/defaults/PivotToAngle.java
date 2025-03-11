package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PivotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
 
public class PivotToAngle extends Command {

    private TrapezoidProfile trapezoidal;
    private Rollers rollers = Rollers.getInstance();
    private JS js = JS.getInstance();
    double setPoint;
    private double angle;


    public void initialize() {
        js.setGoal(angle);
        js.setSetpoint(new TrapezoidProfile.State(js.getPivotPosition(), 0.0));
    }

    public PivotToAngle(JS pivot, Rollers rollers, double angle) {
        this.angle = angle;

        addRequirements(js);

    }

    public void execute() {
        js.setLastGoal(angle);
        js.setPower(0);
        trapezoidal = new TrapezoidProfile(
            js.getConstraints());

        double pose = js.getPivotPosition();
        TrapezoidProfile.State next = trapezoidal.calculate(0.02, js.getSetpoint(), js.getGoal());
        js.setSetpoint(next);
        js.getPID().setSetpoint(next.position);
        double power = js.getPID().calculate(pose);
        SmartDashboard.putNumber("power", power);
        double PIDFFpower = power;
        SmartDashboard.putNumber("PIDFFpower", PIDFFpower);
        js.setPower(-PIDFFpower);
    }

    public void end(boolean interrupted) {
        js.setPower(0);
    }
}

