package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.JSConstants;
import frc.robot.Constants.PivotConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class JS extends SubsystemBase {
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    private PIDController pid;

    private TrapezoidProfile current;
    private TalonFX pivotMotor;
    //private Encoder thru_bore;

    public JS() {
        pivotMotor = new TalonFX(JSConstants.JS_ID);
        //(PivotConstants.pivotMotorID);

        //thru_bore = new Encoder(6, 7);

        constraints = new TrapezoidProfile.Constraints(PivotConstants.maxVelocity, PivotConstants.maxAcceleration);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        // this.ff = new SimpleMotorFeedforward(
        //     0.025, .012 ,.01
        // );


        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
            slot0Configs.kS = 0.7;   
            slot0Configs.kV = 0.1; 
            slot0Configs.kA = 0.1; 
            slot0Configs.kP = PivotConstants.P; 
            slot0Configs.kI = PivotConstants.I; 
            slot0Configs.kD = PivotConstants.D; 

        pivotMotor.getConfigurator().apply(talonFXConfigs);
    }

    public double getPivotPosition() {
        double pivotPose = pivotMotor.getPosition().getValueAsDouble();
        pivotPose = (pivotPose * PivotConstants.GEAR_RATIO);

        return pivotPose;
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

    // public int getEncoder() {
    //     return thru_bore.get();
    // }

    public void zeroEncoders() {
        pivotMotor.setPosition(0);
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public Command zero_command() {
        return this.runOnce(() -> zeroEncoders());
    }

    public Command stop_command() {
        return this.runOnce(() -> setPower(0));
    }

    public Command setSpeed_command(double speed) {
        return this.runOnce(() -> setSpeed(speed));
    }

    public void setPower(double power) {
        pivotMotor.set(power);
    }
            
    public void setGoal(double current) {
        goal = new TrapezoidProfile.State(current, 0.1);
    }

    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint; 
    }

    public void periodic() {
        SmartDashboard.putNumber("encoder pose", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("js pose", getPivotPosition());
    }


  }
