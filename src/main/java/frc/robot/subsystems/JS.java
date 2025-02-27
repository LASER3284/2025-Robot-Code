package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.JSConstants;
import frc.robot.Constants.PivotConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class JS extends SubsystemBase {
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private TrapezoidProfile current;
    private TalonFX pivotMotor;
    private DutyCycleEncoder thru_bore; //= new DutyCycleEncoder(3);

    private TrapezoidProfile current_js;

    private PIDController pid;

    private double rotations;

    public JS() {
        pivotMotor = new TalonFX(JSConstants.JS_ID);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        //(PivotConstants.pivotMotorID);

        thru_bore = new DutyCycleEncoder(3);

        constraints = new TrapezoidProfile.Constraints(60, 60);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        

        // this.ff = new SimpleMotorFeedforward(
        //     0.025, .012 ,.01
        // );

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
            slot0Configs.kS = 0.1;   
            slot0Configs.kV = 0.5; 
            slot0Configs.kA = 0.005; 
            slot0Configs.kG = 0.8;
            slot0Configs.kP = 0.4; 
            slot0Configs.kI = 0; 
            slot0Configs.kD = 0;
        
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; 
        motionMagicConfigs.MotionMagicAcceleration = 20; 
        motionMagicConfigs.MotionMagicJerk = 1600;

        pivotMotor.getConfigurator().apply(talonFXConfigs);
        pivotMotor.getConfigurator().apply(motionMagicConfigs);


        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        zeroEncoders();

    }

    public double getPivotPosition() {
        double pivotPose = thru_bore.get();
        pivotPose = (pivotPose * 0.0107146684);
        System.out.println(thru_bore.get());
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

    public void setPivotPosition(double rotations) {

        final MotionMagicVoltage m_request = new MotionMagicVoltage(rotations);

        // set target position to 100 rotations
        pivotMotor.setControl(m_request.withPosition(rotations));

        this.rotations = rotations;
    }

    public Command JSCommand(double rotations) {
        return this.runOnce(() -> setPivotPosition(rotations));
    }

    public double getEncoder() {
        return thru_bore.get();
    }

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
        SmartDashboard.putNumber("js pose", getEncoder());
    }


  }
