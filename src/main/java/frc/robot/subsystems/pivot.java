package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class pivot extends SubsystemBase {
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    private PIDController pid;
    //private SimpleMotorFeedforward ff;
    private TrapezoidProfile current;
    private TalonFX pivotMotor;

    public pivot() {
        pivotMotor = new TalonFX(22);
        //(PivotConstants.pivotMotorID);

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

    public void zeroEncoders() {
        pivotMotor.setPosition(0);
    }

    public Command zero_command() {
        return this.runOnce(() -> zeroEncoders());
    }

    public Command stop_command() {
        return this.runOnce(() -> setPower(0));
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
        SmartDashboard.putNumber("right motor pose", pivotMotor.getPosition().getValueAsDouble());
    }


  }
