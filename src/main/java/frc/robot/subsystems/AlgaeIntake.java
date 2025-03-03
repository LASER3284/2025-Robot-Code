package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.commands.algae_intake.AlgaeDeploy;

public class AlgaeIntake extends SubsystemBase {
    private static AlgaeIntake instance;
    
    private SparkFlex rackmotor;
    private TalonFX rollermotor;
    private DigitalInput limit;

    private TrapezoidProfile current;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private PIDController pid;

    private Distance last_goal;
    private double last_speed;


    public AlgaeIntake() {
        rackmotor = new SparkFlex(AlgaeIntakeConstants.ARACK_ID, MotorType.kBrushless);
        rollermotor = new TalonFX(AlgaeIntakeConstants.AROLLER_ID);

        limit = new DigitalInput(0);
            
        constraints = new TrapezoidProfile.Constraints(460, 460);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        pid = new PIDController(0.095, 0, 0.0);

        last_goal = Inches.of(-9);

        var talonFXConfigs = new TalonFXConfiguration();

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50; 
        motionMagicConfigs.MotionMagicAcceleration = 90; 
        motionMagicConfigs.MotionMagicJerk = 200; 

        rollermotor.getConfigurator().apply(motionMagicConfigs);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new AlgaeDeploy(this, last_goal));
    }

    public static AlgaeIntake getInstance() {
        if (instance == null) {
            instance = new AlgaeIntake();
        }
        return instance;
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
        return this.runOnce(() -> setRollerSpeed(-speed));
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

    public PIDController getPID() {
        return pid;
    }

    public boolean isAtSetpoint(double goal) {
        return (getAlgaePosition() - goal) < 0.1;
    }
    
    // SETTERS \\

    public void setGoal(double goal_pose) {
        goal = new TrapezoidProfile.State(goal_pose,0);
    }
    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint;
    }

    public void setRollerSpeed(double speed) {
        rollermotor.set(-speed);
    }

    public void setRackSpeed(double speed) {
        rackmotor.set(speed);
    }

    public void setLastGoal(Distance goal) {
        last_goal = goal;
    }

    // SYSID \\


    public void periodic() {
        //SmartDashboard.putNumber("distance", encoder.getDistance());
        SmartDashboard.putNumber("getalgae pose", getAlgaePosition());
        SmartDashboard.putNumber("motor dist", rackmotor.getEncoder().getPosition());
        //SmartDashboard.putNumber("DISTANCE TO POSE", getAlgaePosition() - )

        initDefaultCommand();
    }   
}
