package frc.robot.commands.algae_intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeDeployEnd extends Command {
    private final AlgaeIntake algaeintake;
    private TrapezoidProfile current;

    private Distance extension_length;

    public void initialize() {
        algaeintake.setGoal(extension_length.magnitude());

        algaeintake.setSetpoint(
            new TrapezoidProfile.State(algaeintake.getAlgaePosition(), 0.0));
    }

    public AlgaeDeployEnd(AlgaeIntake algaeintake, Distance extension_length) {
        this.algaeintake = algaeintake;
        this.extension_length = extension_length;

        // this.feedforward = new ElevatorFeedforward(0.002,
        // 0.05,
        // 2.45,
        // 0.02);

        //this.rackPID = new PIDController(0.1, 0, 0.0025);
        addRequirements(algaeintake);
    }

    public void execute() {
        //algaeintake.setRollerSpeed(-0.5);

        algaeintake.setLastGoal(extension_length);
        current = new TrapezoidProfile(algaeintake.getConstraints());
        double position = algaeintake.getAlgaePosition();

        TrapezoidProfile.State new_goal = current.calculate(0.02, algaeintake.getSetpoint(), algaeintake.getGoal());

        algaeintake.setSetpoint(new_goal);

        algaeintake.getPID().setSetpoint(new_goal.position);
        double power = algaeintake.getPID().calculate(position);

        algaeintake.setRackSpeed(power);
    }

    public void end(boolean interrupted) {
        algaeintake.setRackSpeed(0);
    }

    public boolean isFinished() {
        SmartDashboard.putBoolean("isFinished", Math.abs(algaeintake.getAlgaePosition() - extension_length.magnitude()) < 1);
        SmartDashboard.putNumber("isFinished math", Math.abs(algaeintake.getAlgaePosition() - extension_length.magnitude()));
        
        return Math.abs(algaeintake.getAlgaePosition() - extension_length.magnitude()) < 1;
    }
}
