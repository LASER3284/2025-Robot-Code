package frc.robot.subsystems.pivotintake;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private TalonFX Pivotmotor;
    
    //private Encoder encoder;
    
    private PIDController pivotPID;
    private SimpleMotorFeedforward ff;
    private TrapezoidProfile current;

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    
    
    public Pivot() {
        Pivotmotor = new TalonFX(15);
        //encoder = new Encoder(0, 1);

        constraints = new TrapezoidProfile.Constraints(10, 10);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        this.ff = new SimpleMotorFeedforward(
            0.006, 0.012, 0.5679
        );
        this.pivotPID = new PIDController(
            0.1,0,0
        );
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

    public double getPivotPosition() {
        double pivotpose = (Pivotmotor.getPosition().getValueAsDouble());
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
    }
}
