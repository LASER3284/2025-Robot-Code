package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {
    private SparkFlex rackmotor;
    private TalonFX rollermotor;
    private DigitalInput limit;
    private DigitalInput sensor;

    private TrapezoidProfile current;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private ProfiledPIDController pid;
   
    public AlgaeIntake() {
        rackmotor = new SparkFlex(AlgaeIntakeConstants.ARACK_ID, MotorType.kBrushless);
        rollermotor = new TalonFX(AlgaeIntakeConstants.AROLLER_ID);

        limit = new DigitalInput(AlgaeIntakeConstants.DI_LIMIT_PORT);
        sensor = new DigitalInput(AlgaeIntakeConstants.DI_SENSOR_PORT);
            
        constraints = new TrapezoidProfile.Constraints(820, 820);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        pid = new ProfiledPIDController(0.1, 0, 0.0025, constraints);
    }

    // SYSID \\

    public void getEncoder(double pose) {
        rackmotor.getEncoder();
    }

    public void stopMotor() {
        rollermotor.stopMotor();
        rackmotor.stopMotor();
    }

    public void zeroEncoder() {
        rackmotor.getEncoder().setPosition(0);
    }

    // COMMANDS \\

    public Command stopmotor_command()
    {
        return this.runOnce(() -> setRackSpeed(0));
    }

    public Command zero_command() {
        return this.runOnce(() -> zeroEncoder());
    }
public Command rollerSpeed_Command(double speed) {
        return this.runOnce(() -> setRollerSpeed(speed));
    }
    

    // GETTERS \\

    public double getAlgaePosition() {
        double algaepose = rackmotor.getEncoder().getPosition();
        algaepose = (algaepose * (36/15)) * Math.PI * 0.5;
        return algaepose;
    }

    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    public TrapezoidProfile.State getGoal() {
        return goal;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public TrapezoidProfile getCurrent() {
        return current;
    }

    public boolean getLimit() {
        return limit.get();
    }

    public ProfiledPIDController getPID() {
        return pid;
    }

    public boolean getSensor() {
        return sensor.get();
    }

    // SETTERS \\

    public void setGoal(double goal_pose) {
        goal = new TrapezoidProfile.State(goal_pose,0);
    }
    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint;
    }

    public void setRollerSpeed(double speed) {
        rollermotor.set(speed);
    }

    public void setRackSpeed(double speed) {
        rackmotor.set(speed);
    }

    // SYSID \\


    public void periodic() {
        //SmartDashboard.putNumber("distance", encoder.getDistance());
        SmartDashboard.putNumber("getalgae pose", getAlgaePosition());
        SmartDashboard.putNumber("motor dist", rackmotor.getEncoder().getPosition());
        //SmartDashboard.putNumber("DISTANCE TO POSE", getAlgaePosition() - )
    }   
}
