package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.*;
 
public class PivotToAngle extends Command{

    private PIDController pivotPID;
    private Rollers rollers;
    private TrapezoidProfile trappy;
    private JS Pivot;
    double setPoint;
    private Angle angle;
    double speed;


    public void initialize() {
        Pivot.setGoal(angle.magnitude());
        Pivot.setSetpoint(new TrapezoidProfile.State(Pivot.getPivotPosition(), 0.0));
    }

    public  PivotToAngle(JS Pivot, Rollers rollers, Angle angle, double speed) {
        this.Pivot = Pivot;
        this.rollers = rollers;
        this.angle = angle;
        this.speed = speed;

        this.pivotPID = new PIDController(
            0.6, 
            PivotConstants.I, 
            0);
        addRequirements(Pivot, rollers);
    }

    public void execute() {
        rollers.coral_roller_on_command(speed);
        Pivot.setPower(0);
        trappy = new TrapezoidProfile(
            Pivot.getConstraints());
        double pose = Pivot.getPivotPosition();
        TrapezoidProfile.State next = trappy.calculate(0.02, Pivot.getSetpoint(), Pivot.getGoal());
        Pivot.setSetpoint(next);
        pivotPID.setSetpoint(next.position);
        double power = pivotPID.calculate(pose);
        SmartDashboard.putNumber("power", power);
        double PIDFFpower = power;
        SmartDashboard.putNumber("PIDFFpower", PIDFFpower);
        Pivot.setPower(-PIDFFpower);
    }

    public void end(boolean interrupted) {
        Pivot.setPower(0);
    }
        
    public boolean isFinished() { 
        SmartDashboard.putNumber("Angle Magnitude: ", angle.magnitude());
        SmartDashboard.putNumber("distance to goal", Pivot.getPivotPosition() - angle.magnitude());
        return Math.abs(Pivot.getPivotPosition() - angle.magnitude()) < 0.01;
    }
}

