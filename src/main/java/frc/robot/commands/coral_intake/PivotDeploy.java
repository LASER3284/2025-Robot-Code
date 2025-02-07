package frc.robot.commands.coral_intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivotintake.Pivot;

public class PivotDeploy extends Command {
    private final Pivot pivot;
    private TrapezoidProfile current;
    private SimpleMotorFeedforward ff;
    private PIDController pivotPID;
    private Angle degrees;


    public void initialize() {
        pivot.setGoal(degrees.magnitude());

        pivot.setSetpoint(
            new TrapezoidProfile.State(pivot.getPivotPosition(), 0));
    }

    public PivotDeploy(Pivot pivot, Angle degrees) {
        this.pivot = pivot;
        this.degrees = degrees;

        this.ff = new SimpleMotorFeedforward(0.1,0.27,0.3);
            
        this.pivotPID = new PIDController(.1,0,.001);
        addRequirements(pivot);
    }

    public void execute() {
        pivot.setPower(0.1);
        current = new TrapezoidProfile(
            pivot.getConstraints());
        double pose = pivot.getPivotPosition();
        SmartDashboard.putNumber("goal", pivot.getGoal().position);
        TrapezoidProfile.State next = current.calculate(0.02, pivot.getSetpoint(), pivot.getGoal());
        double ff_power = ff.calculate(next.velocity) / 12;
        pivot.setSetpoint(next);
        SmartDashboard.putNumber("setpoint", pivot.getSetpoint().position);
        pivotPID.setSetpoint(next.position);
        double power = pivotPID.calculate(pose);
        double PIDFFpower = power + ff_power;
        pivot.setPower(PIDFFpower);
    }

    public void end(boolean interrupted) {
        pivot.setPower(0.1);
    }

    public boolean isFinished() {
        SmartDashboard.putBoolean("isFinihsed", Math.abs(pivot.getPivotPosition() - degrees.magnitude()) < 0);
        SmartDashboard.putNumber("isFinished math", Math.abs(pivot.getPivotPosition() - degrees.magnitude()));
        return Math.abs(pivot.getPivotPosition() - degrees.magnitude()) < 0.1;
    }
}
