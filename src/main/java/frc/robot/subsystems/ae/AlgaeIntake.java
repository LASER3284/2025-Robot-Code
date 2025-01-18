package frc.robot.subsystems.ae;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlgaeIntake extends SubsystemBase {
    private SparkFlex rack_motor;
    private SparkMax roller_motor;

    private TrapezoidProfile current_pose;
    private ElevatorFeedforward ff;
    private PIDController rackPID;

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    
    public AlgaeIntake() {
        rack_motor = new SparkFlex(0, MotorType.kBrushless);
        roller_motor = new SparkMax(0, MotorType.kBrushless);

        constraints = new TrapezoidProfile.Constraints(AlgaeIntakeConstants.maxVelocity, AlgaeIntakeConstants.maxAcceleration);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        this.ff = new ElevatorFeedforward(
            AlgaeIntakeConstants.kS, 
            AlgaeIntakeConstants.kG, 
            AlgaeIntakeConstants.kV);
        this.rackPID = new PIDController(
            AlgaeIntakeConstants.P, 
            AlgaeIntakeConstants.I, 
            AlgaeIntakeConstants.D);
    }

    public double getIntakePosition() {
        double intakepose = rack_motor.getExternalEncoder().getPosition();
        intakepose = (intakepose * AlgaeIntakeConstants.GEAR_RATIO);

        return intakepose;
    }

    public void zeroEncoder() {
        rack_motor.getExternalEncoder().setPosition(0);
    }

    // GETTERS \\
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    public TrapezoidProfile.State getGoal() {
        return goal;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public PIDController getPID() {
        return rackPID;
    }

    public ElevatorFeedforward getFF() {
        return ff;
    }

    public TrapezoidProfile getCurrent() {
        return current_pose;
    }

    public double getRollerSpeed() {
        return roller_motor.get();
    }

    // SETTERS \\
    public void setPose(double new_pose) {
        rack_motor.getExternalEncoder().setPosition(new_pose);
    }

    public void setGoal(double goal_pose) {
        goal = new TrapezoidProfile.State(goal_pose, 0);
    }

    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint; 
    }

    public void setRollerSpeed(double speed) {
        roller_motor.set(speed);
    }

    public void setRackSpeed(double speed) {
        rack_motor.set(speed);
    }

    // COMMANDS \\
    public Command zero_encoders_command() {
        return this.runOnce(() -> zeroEncoder());
    }

    public Command set_pose_command(double new_pose) {
        return this.runOnce(() -> setPose(new_pose));
    }

    public void periodic() {

    }
}
