// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;



public class Elevator extends SubsystemBase {
    public static double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    
    private TalonFX elevatorMotor;
    private TalonFX carriageMotor;

    private DigitalInput limit;

    private TrapezoidProfile current;
    DutyCycleOut pose;

    private final TrapezoidProfile.Constraints e_constraints;
    private TrapezoidProfile.State e_goal;
    private TrapezoidProfile.State e_setpoint;

    private final TrapezoidProfile.Constraints c_constraints;
    private TrapezoidProfile.State c_goal;
    private TrapezoidProfile.State c_setpoint;
        
    public Elevator() {
        elevatorMotor = new TalonFX(ElevatorConstants.ER_ID);
        carriageMotor = new TalonFX(ElevatorConstants.EL_ID);
        
        e_constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
        e_goal = new TrapezoidProfile.State();
        e_setpoint = new TrapezoidProfile.State();

        c_constraints = new TrapezoidProfile.Constraints(1, 1);
        e_goal = new TrapezoidProfile.State();
        e_setpoint = new TrapezoidProfile.State();
        
        var talonFXConfigs = new TalonFXConfiguration();
    
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; 
        slot0Configs.kV = 0.12; 
        slot0Configs.kA = 0.01; 
        slot0Configs.kP = ElevatorConstants.P; 
        slot0Configs.kI = ElevatorConstants.I; 
        slot0Configs.kD = ElevatorConstants.D; 
    
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 10; 
        motionMagicConfigs.MotionMagicAcceleration = 20; 
        motionMagicConfigs.MotionMagicJerk = 1600;
    
        elevatorMotor.getConfigurator().apply(talonFXConfigs);
    }

    public double getElevatorPosition() {
        double elevatorpose = (elevatorMotor.getPosition().getValueAsDouble());
        elevatorpose = (elevatorpose * ElevatorConstants.GEAR_RATIO) * ElevatorConstants.LINEAR_DISTANCE_CONST;

        return elevatorpose;
    }

    public double getCarriagePosition() {
        double carriagepose = (carriageMotor.getPosition().getValueAsDouble());
        carriagepose = (carriagepose * ElevatorConstants.GEAR_RATIO) * ElevatorConstants.LINEAR_DISTANCE_CONST;

        return carriagepose;
    }

    public void zeroEncoders() {
        elevatorMotor.setPosition(0);
        carriageMotor.setPosition(0);
    }

    // COMMANDS \\
    public Command zero_command() {
        return this.runOnce(() -> zeroEncoders());
    }

    public Command stop_ele_command() {
        return this.runOnce(() -> setElevatorPower(0));
    }

    public Command stop_car_command() {
        return this.runOnce(() -> setCarriagePower(0));
    }

    public Command set_ele_power_command(double power) {
        return this.runOnce(() -> setElevatorPower(power));
    }

    public Command set_car_power_command(double power) {
        return this.runOnce(() -> setCarriagePower(power));
    }
    
    // GETTERS \\
    public TrapezoidProfile.State getSetpoint(String choice) {
        if (choice.equals("elevator")) { return e_setpoint; }
        else if (choice.equals("carriage")) { return c_setpoint; }
        else return null;
    }

    public TrapezoidProfile.State getGoal(String choice) {
        if (choice.equals("elevator")) { return e_goal; }
        else if (choice.equals("carriage")) { return c_goal; }
        else return null;
    }

    public TrapezoidProfile.Constraints getConstraints(String choice) {
        if (choice.equals("elevator")) { return e_constraints; }
        else if (choice.equals("carriage")) { return c_constraints; }
        else return null;
    }

    public TrapezoidProfile getCurrent() {
        return current;
    }

    public boolean getLimit() {
        return limit.get();
    }

    // SETTERS \\
        
    public void setElevatorPower(double power) {
        elevatorMotor.set(power);
    }

    public void setCarriagePower(double power) {
        carriageMotor.set(power);
    }
            
    public void setEGoal(double goal_pose) {
        e_goal = new TrapezoidProfile.State(goal_pose, 0);
    }

    public void setESetpoint(TrapezoidProfile.State setpoint) {
        this.e_setpoint = setpoint; 
    }

    public void setCGoal(double goal_pose) {
        c_goal = new TrapezoidProfile.State(goal_pose, 0);
    }

    public void setCSetpoint(TrapezoidProfile.State setpoint) {
        this.c_setpoint = setpoint; 
    }

    public void setDrivetrainSpeed() {
        if (getElevatorPosition() > 12) {
            MaxSpeed = 1;
            MaxAngularRate = 1;
        } else if (getElevatorPosition() > 24) {
            MaxSpeed = 2;
            MaxAngularRate = 2;
        } else if (getElevatorPosition() > 36) {
            MaxSpeed = 3;
            MaxAngularRate = 3;
        } else if (getElevatorPosition() > 48) {
            MaxSpeed = 4;
            MaxAngularRate = 4;
        } else if (getElevatorPosition() > 60) {
            MaxSpeed = 5;
            MaxAngularRate = 5;
        } else {
            MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
            MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
        }
    }                           

    public void periodic() {
        SmartDashboard.putNumber("elevator motor pose", elevatorMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("carriage motor pose", carriageMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator pose", getElevatorPosition());
        SmartDashboard.putNumber("drive speed", MaxSpeed);
    }
}
