// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    public enum States {
        HANDOFF("HANDOFF", ElevatorConstants.HANDOFF_HEIGHT),
        L2("L2", ElevatorConstants.L2_HEIGHT),
        L3("L3", ElevatorConstants.L3_HEIGHT),
        L4("L4", ElevatorConstants.L4_HEIGHT);

        private String state;
        private Distance height;
    
        <Inches> States(String state, Distance height) {
            this.state = state;
            this.height = height;
        }

        public double getHeight() {
            return height.baseUnitMagnitude();
        }
    }

    DutyCycleOut pose;

    private TalonFX rightMotor;
    private TalonFX leftMotor;

    // test stuff
    private TrapezoidProfile current;
    private ElevatorFeedforward ff;
    private PIDController elevatorPID;
    //test stuff

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    
    private States state;
    
    private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        
    public Elevator() {
        rightMotor = new TalonFX(ElevatorConstants.ER_ID);
        leftMotor = new TalonFX(ElevatorConstants.EL_ID);
        
        constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();
        
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
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
        rightMotor.getConfigurator().apply(talonFXConfigs);
        leftMotor.getConfigurator().apply(talonFXConfigs);

        //testing stuff
        this.ff = new ElevatorFeedforward(
            ElevatorConstants.kS, 
            ElevatorConstants.kG, 
            ElevatorConstants.kV);
        this.elevatorPID = new PIDController(
            ElevatorConstants.P, 
            ElevatorConstants.I, 
            ElevatorConstants.D);
        // testing stuff
    }
    
    public void setPower(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
    }
            
    public void setGoal(double goal_pose) {
        goal = new TrapezoidProfile.State(goal_pose, 0);
    }

    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint; 
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

    public double getElevatorPosition() {
        double elevatorpose = (rightMotor.getPosition().getValueAsDouble() + leftMotor.getPosition().getValueAsDouble()) / 2;
        elevatorpose = (elevatorpose * ElevatorConstants.GEAR_RATIO) * ElevatorConstants.LINEAR_DISTANCE_CONST;

        return elevatorpose;
    }

    public void zeroEncoders() {
        rightMotor.setPosition(0);
        leftMotor.setPosition(0);
    }

    public void toPositionL2() {
        rightMotor.setVoltage(elevatorPID.calculate(getElevatorPosition() / 0.5, 2) + ff.calculate(0));
        leftMotor.setVoltage(elevatorPID.calculate(getElevatorPosition() / 0.5, 2) + ff.calculate(0));
        SmartDashboard.putNumber("right motor voltage", rightMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("right motor voltage", leftMotor.getMotorVoltage().getValueAsDouble());
        
    }

    // COMMANDS \\
    public Command zero_command() {
        return this.runOnce(() -> zeroEncoders());
    }

    public Command stop_command() {
        return this.runOnce(() -> setPower(0));
    }

    public Command toPositionL2Command() {
        return Commands.runOnce( ()-> toPositionL2());
    }

    //test
    public Command goToL2() {
        return startRun(
            () -> {
                setGoal(ElevatorConstants.L2_HEIGHT.baseUnitMagnitude());
                System.out.println(getGoal());

                setSetpoint(
                new TrapezoidProfile.State(getElevatorPosition(), 0.0));
                System.out.println(getSetpoint());
            },
            () -> {
                setPower(0);
                current = new TrapezoidProfile(
                    getConstraints());
        
                double pose = getElevatorPosition();

                TrapezoidProfile.State next = current.calculate(0.02, getSetpoint(), getGoal());

                double ff_power = ff.calculate(next.velocity) / 12;

                setSetpoint(next);

                elevatorPID.setSetpoint(next.position);

                // testing
                System.out.println(getSetpoint());
                System.out.println(elevatorPID.getSetpoint());

                double power = elevatorPID.calculate(pose);

                double PIDFFpower = power + ff_power;
                // testing
                SmartDashboard.putNumber("PIDFFpower", PIDFFpower);

                setPower(PIDFFpower);
            }
        );
    }
    //test

    public void periodic() {
        SmartDashboard.putNumber("right motor pose", rightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("left motor pose", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator pose", getElevatorPosition());
        SmartDashboard.putNumber("feed forward calc", ff.calculate(0));

        //testing
        SmartDashboard.putNumber("L2 goal", ElevatorConstants.L2_HEIGHT.magnitude());


        System.out.println(rightMotor.getPosition());
        System.out.println(leftMotor.getPosition());
    }
    }

