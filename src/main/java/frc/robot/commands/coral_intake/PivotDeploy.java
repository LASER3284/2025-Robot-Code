package frc.robot.commands.coral_intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;

public class PivotDeploy extends Command {
    private final Pivot pivot;
    private final IntakeRollers rollers;
    private TrapezoidProfile current;
    private SimpleMotorFeedforward ff;
    private PIDController pivotPID;
    private Angle degrees;



    public void initialize() {
        pivot.setGoal(degrees.magnitude());

        pivot.setSetpoint(
            new TrapezoidProfile.State(pivot.getPivotPosition(), 0));

    }

    public PivotDeploy(Pivot pivot, IntakeRollers rollers, Angle degrees) {
        this.pivot = pivot;
        this.rollers = rollers;
        this.degrees = degrees;
            
        this.pivotPID = new PIDController(.4, 0, 0.005);
        addRequirements(pivot);
    }

    public void execute() {
        pivot.setPower(0);
        current = new TrapezoidProfile(
            pivot.getConstraints());
        double pose = pivot.getPivotPosition();
        SmartDashboard.putNumber("goal", pivot.getGoal().position);
        TrapezoidProfile.State next = current.calculate(0.02, pivot.getSetpoint(), pivot.getGoal());
        pivot.setSetpoint(next);
        SmartDashboard.putNumber("setpoint", pivot.getSetpoint().position);
        pivotPID.setSetpoint(next.position);
        double power = pivotPID.calculate(pose);
        pivot.setPower(power);
    }

    public void end(boolean interrupted) {
        pivot.setPower(0);
    }

    public boolean isFinished() {
        SmartDashboard.putBoolean("isFinished", Math.abs(pivot.getPivotPosition() - degrees.magnitude()) < 0);
        SmartDashboard.putNumber("isFinished math", Math.abs(pivot.getPivotPosition() - degrees.magnitude()));
        return Math.abs(pivot.getPivotPosition() - degrees.magnitude()) < 0.1;
    }
}
