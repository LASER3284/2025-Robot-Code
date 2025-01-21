package frc.robot.commands.algae;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ae.AlgaeIntake;

public class AlgaeDeploy extends Command {
    private final AlgaeIntake algaeintake;
    private TrapezoidProfile current;
    private ElevatorFeedforward ff;
    private PIDController algaePID;

    public void initialize() {
        algaeintake.setgoal(AlgaeIntakeConstants.DEPLOYED.magnitude());

        algaeintake.setsetpoint(
            new TrapezoidProfile.State(algaeintake.getAlgaePosition(), 0.0));
    }

    public AlgaeDeploy(AlgaeIntake algaeintake) {
        this.algaeintake = algaeintake;
        this.ff = new ElevatorFeedforward(
            AlgaeIntakeConstants.kS, 
            AlgaeIntakeConstants.kG, 
            AlgaeIntakeConstants.kV);
        this.algaePID = new PIDController(
            AlgaeIntakeConstants.P, 
            AlgaeIntakeConstants.I, 
            AlgaeIntakeConstants.D);
        addRequirements(algaeintake);
    }

    public void execute() {
        algaeintake.setRollerSpeed(4);
        algaeintake.setRackSpeed(0);
        current = new TrapezoidProfile(
            algaeintake.getconstraints());
        double pose = algaeintake.getAlgaePosition();
        TrapezoidProfile.State next = current.calculate(0.02, algaeintake.getSetpoint(), algaeintake.getgoal());
        double ff_power = ff.calculate(next.velocity) / 12;
        algaeintake.setsetpoint(next);
        algaePID.setSetpoint(next.position);
        double power = algaePID.calculate(pose);
        double PIDFFpower = power + ff_power;
        algaeintake.setRackSpeed(PIDFFpower);

    }


}
