package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.JSConstants;
import frc.robot.Constants.PivotConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class JS extends SubsystemBase {
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private TrapezoidProfile current;
    private TalonFX pivotMotor;
    private DutyCycleEncoder thru_bore; //= new DutyCycleEncoder(3);

    private ArmFeedforward ff;
    
    private TrapezoidProfile current_js;

    private PIDController pid;

    public double current_pose;

    public JS() {
        pivotMotor = new TalonFX(JSConstants.JS_ID);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        //(PivotConstants.pivotMotorID);

        thru_bore = new DutyCycleEncoder(3);

        constraints = new TrapezoidProfile.Constraints(60, 60);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        this.ff = new ArmFeedforward(
            0.025, .3 ,.01, 0.013
        );

        current_pose = thru_bore.get();

        pid = new PIDController(2, 0, 0);
        
        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 10; 
        // motionMagicConfigs.MotionMagicAcceleration = 20; 
        // motionMagicConfigs.MotionMagicJerk = 1600;

        // pivotMotor.getConfigurator().apply(talonFXConfigs);

        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getPivotPosition() {
        double pivotPose = thru_bore.get();
        //pivotPose = (pivotPose * 0.0107146684);
        //System.out.println(thru_bore.get());
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

    public PIDController getPID() {
        return pid;
    }

    // public int getEncoder() {
    //     return thru_bore.get();
    // }

    public void zeroEncoders() {
        //thru_bore.set;
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

    public boolean isSafeGroundIn() {
        return thru_bore.get() < 0.75 && thru_bore.get() > 0.55;
    }

    public boolean isAtSetpoint(double angle) {
        return (thru_bore.get() - angle) < 0.1;
    }

    public void calculateJSPose(double angle) {
        setGoal(angle);
        setSetpoint(
            new TrapezoidProfile.State(getPivotPosition(), 0.0));
        
        current_js = new TrapezoidProfile(
            getConstraints());
        double cpose = getPivotPosition();
        TrapezoidProfile.State cnext = current_js.calculate(0.02, getSetpoint(), getGoal());
         double cff_power = ff.calculate(cnext.position, cnext.velocity);
        setSetpoint(cnext);
        getPID().setSetpoint(cnext.position);
        double cpower = getPID().calculate(cpose);
        double ctotalpower = cpower + cff_power;

        SmartDashboard.putNumber("ctotalpower", ctotalpower);

        pivotMotor.set(ctotalpower);
    }

    public void setPose(double angle) {
        current_pose = angle;
    }

    public Command setPoseCommand(double angle) {
        return this.run(() -> setPose(angle));
    }

    public Command calcCommand(double angle) {
        return this.run(() -> calculateJSPose(angle));
    }

    public void periodic() {
        SmartDashboard.putNumber("encoder pose", thru_bore.get());
        SmartDashboard.putNumber("js pose", getPivotPosition());//getPivotPosition());
        SmartDashboard.putNumber("current pose", current_pose);

        calculateJSPose(current_pose);

    }


  }
