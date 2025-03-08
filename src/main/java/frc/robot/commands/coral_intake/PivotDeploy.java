package frc.robot.commands.coral_intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivotintake.Pivot;

public class PivotDeploy extends Command {
    private Pivot pivot = Pivot.getInstance();
    private TrapezoidProfile current;
    private double degrees;
    
    
    
    public void initialize() {
        pivot.setGoal(degrees);
    
        pivot.setSetpoint(
            new TrapezoidProfile.State(pivot.getPivotPosition(), 0));
    }
    
    public PivotDeploy(Pivot pivot, double degrees) {
        this.pivot = pivot;
        this.degrees = degrees;
        
        addRequirements(pivot);
    }

    public void execute() {
        pivot.setPower(0);

        pivot.setLastGoal(degrees);
        current = new TrapezoidProfile(
            pivot.getConstraints());
        double pose = pivot.getPivotPosition();
        SmartDashboard.putNumber("goal", pivot.getGoal().position);
        TrapezoidProfile.State next = current.calculate(0.02, pivot.getSetpoint(), pivot.getGoal());
        pivot.setSetpoint(next);
        SmartDashboard.putNumber("setpoint", pivot.getSetpoint().position);
        pivot.getPID().setSetpoint(next.position);
        double power = pivot.getPID().calculate(pose);
        pivot.setPower(-power);
    }

    public void end(boolean interrupted) {
        pivot.setPower(0);
    }

    
}
