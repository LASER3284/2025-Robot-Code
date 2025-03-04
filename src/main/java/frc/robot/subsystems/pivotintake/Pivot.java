package frc.robot.subsystems.pivotintake;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.coral_intake.PivotDeploy;

public class Pivot extends SubsystemBase {
    private static Pivot instance;

    private TalonFX Pivotmotor;
    
    private DutyCycleEncoder encoder;
    
    private PIDController pivotPID;
    private SimpleMotorFeedforward ff;
    private TrapezoidProfile current;

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    double last_goal;
    
    
    public Pivot() {
        Pivotmotor = new TalonFX(15);
        encoder = new DutyCycleEncoder(2);

        constraints = new TrapezoidProfile.Constraints(300, 300);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        this.ff = new SimpleMotorFeedforward(
            0.006, 0.012, 0.5679
        );
        this.pivotPID = new PIDController(
            0.75,0,0
        );

        zeroEncoders();

        //encoder.setDutyCycleRange(0, 360);

        last_goal = 0.23;
   }

   public static Pivot getInstance() {
    if (instance ==null) {
        instance = new Pivot();
    }
    return instance;
   }

   public void initDefaultCommand() {
        setDefaultCommand(new PivotDeploy(this, last_goal));
   }

   public void setMotorSpeed(double speed){
        Pivotmotor.set(speed);
   }

   public void stopMotor() {
        Pivotmotor.stopMotor();
   }

    public void zeroPivot(){
        Pivotmotor.setPosition(0);
    }

    public boolean isAtSetpoint(double angle) {
        return Math.abs((encoder.get() - angle)) < 0.09;
    }


    public double getPivotPosition() {
        double pivotpose = (encoder.get());
        return pivotpose;
    }

    public void zeroEncoders() {
        Pivotmotor.setPosition(0);
    }

    public void stop(){
        Pivotmotor.stopMotor();
   }
   


    public Command setMotorSpeed() {
        return this.runOnce(() -> setMotorSpeed());
    }

    public Command zero_Command() {
        return this.runOnce(() -> zeroEncoders());
    }

    public Command stop_Command() {
        return this.runOnce(() -> stopMotor());
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
        return pivotPID;
    }

    public SimpleMotorFeedforward getff() {
        return ff;
    }

    public TrapezoidProfile getcurrent() {
        return current;
    }

    public void setLastGoal(double angle) {
        this.last_goal = angle;
    }

    
    public double getLastGoal() {
        return last_goal;
    }

    public Command setGoalPose() {
        return this.runOnce(() -> setLastGoal(last_goal));
    }

    public Command setGoalPose(double pose) {
        return this.runOnce(() -> setLastGoal(pose));
    }

    // SETTERS \\

    public void setPower(double power) {
        Pivotmotor.set(power);
    }

    public void setGoal( double goal_pose) {
        goal = new TrapezoidProfile.State(goal_pose, 0);
    }

    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint;
    }

    public void periodic() {
        SmartDashboard.putNumber("Pivot motor pose", getPivotPosition());
        SmartDashboard.putNumber("PIVOT pose", encoder.get());
        SmartDashboard.putBoolean("is connected", encoder.isConnected());

        initDefaultCommand();
    }
}

