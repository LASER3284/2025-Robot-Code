package frc.robot.commands.algae_intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeStow extends Command {
    private final AlgaeIntake algaeintake;
    private TrapezoidProfile current;

    private PIDController rackPID;
    private Distance extension_length;

    public void initialize() {
        algaeintake.setGoal(extension_length.magnitude());

        algaeintake.setSetpoint(
            new TrapezoidProfile.State(algaeintake.getAlgaePosition(), 0.0));
    }

    public AlgaeStow(AlgaeIntake algaeintake, Distance extension_length) {
        this.algaeintake = algaeintake;
        this.extension_length = extension_length;

        this.rackPID = new PIDController(0.1, 0, 0.0015);

        addRequirements(algaeintake);
    }

    public void execute() {

        algaeintake.setRollerSpeed(-0.1);
        current = new TrapezoidProfile(algaeintake.getConstraints());
        double position = algaeintake.getAlgaePosition();

        TrapezoidProfile.State new_goal = current.calculate(0.02, algaeintake.getSetpoint(), algaeintake.getGoal());

        algaeintake.setSetpoint(new_goal);

        rackPID.setSetpoint(new_goal.position);
        double power = rackPID.calculate(position);

        algaeintake.setRackSpeed(-power);

    }

    public void end(boolean interrupted) {
        algaeintake.setRackSpeed(0);
        algaeintake.setRollerSpeed(0);
        algaeintake.zero_command();
    }

    public boolean isFinished() {
        SmartDashboard.putBoolean("isFinished", Math.abs(algaeintake.getAlgaePosition() - extension_length.magnitude()) < 0.1);
        SmartDashboard.putNumber("isFinished math", Math.abs(algaeintake.getAlgaePosition() - extension_length.magnitude()));
        return algaeintake.getLimit();
    }
}
