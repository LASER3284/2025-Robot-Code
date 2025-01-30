package frc.robot.commands.algae_intake;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeStow extends Command {
    private final AlgaeIntake algaeintake;
    private TrapezoidProfile current;
    private ElevatorFeedforward feedforward;
    private PIDController rackPID;
    private Distance extension_length;

    public void initialize() {
        algaeintake.setgoal(extension_length.magnitude());

        algaeintake.setsetpoint(
            new TrapezoidProfile.State(algaeintake.getAlgaePosition(), 0.0));
    }

    public AlgaeStow(AlgaeIntake algaeintake, Distance extension_length) {
        this.algaeintake = algaeintake;
        this.extension_length = extension_length;

        //this.feedforward = new ElevatorFeedforward(0.002, 0.05, 2.45, 0.02);
        this.rackPID = new PIDController(0.1, 0, 0.0015);

        addRequirements(algaeintake);
    }

    public void execute() {
        algaeintake.setRollerSpeed(-0.1);
        current = new TrapezoidProfile(algaeintake.getconstraints());
        double position = algaeintake.getAlgaePosition();
        SmartDashboard.putNumber("goal", algaeintake.getgoal().position);
        TrapezoidProfile.State new_goal = current.calculate(0.02, algaeintake.getSetpoint(), algaeintake.getgoal());
        SmartDashboard.putNumber("new goal val", new_goal.position);
        //double feed_power = feedforward.calculate(new_goal.velocity) / 12;
        algaeintake.setsetpoint(new_goal);
        SmartDashboard.putNumber("setpoint", algaeintake.getSetpoint().position);
        rackPID.setSetpoint(new_goal.position);
        double power = rackPID.calculate(position);
        double PIDFFpower = power;
        SmartDashboard.putNumber("pidffpower", PIDFFpower);
        algaeintake.setRackSpeed(PIDFFpower);
    }

    public void end(boolean interrupted) {
        algaeintake.setRackSpeed(0);
        algaeintake.setRollerSpeed(0);
    }

    public boolean isFinished() {
        SmartDashboard.putBoolean("isFinished", Math.abs(algaeintake.getAlgaePosition() - extension_length.magnitude()) < 0.1);
        SmartDashboard.putNumber("isFinished math", Math.abs(algaeintake.getAlgaePosition() - extension_length.magnitude()));
        return algaeintake.getlimit();
    }
}
