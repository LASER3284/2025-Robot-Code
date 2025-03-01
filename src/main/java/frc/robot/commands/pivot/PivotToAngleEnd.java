package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PivotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
 
public class PivotToAngleEnd extends Command {

    private TrapezoidProfile trappy;
    private Rollers rollers;
    private JS Pivot = JS.getInstance();
    double setPoint;
    private double angle;
    double speed;


    public void initialize() {
        Pivot.setGoal(angle);
        Pivot.setSetpoint(new TrapezoidProfile.State(Pivot.getPivotPosition(), 0.0));
    }

    public PivotToAngleEnd(JS pivot, Rollers rollers, double angle, double cspeed, double aspeed) {
        //Pivot = new JS(rollers);
        this.angle = angle;
        this.speed = speed;

        addRequirements(Pivot);

    }

    public void execute() {
        //rollers.coral_roller_on_command(speed);
        Pivot.setLastGoal(angle);
        Pivot.setPower(0);
        trappy = new TrapezoidProfile(
            Pivot.getConstraints());

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

    public boolean isFinished() {
        return (Math.abs(Pivot.getPivotPosition() - angle )) < 0.1;
    }
}

