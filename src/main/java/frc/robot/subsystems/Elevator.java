package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    public enum States {
        HANDOFF("HANDOFF", ElevatorConstants.HANDOFF_HEIGHT),
        FEED("FEED", ElevatorConstants.FEED_HEIGHT),
        L2("L2", ElevatorConstants.L2_HEIGHT),
        L3("L3", ElevatorConstants.L3_HEIGHT),
        L4("L4", ElevatorConstants.L4_HEIGHT);

        private String state;
        private double height;
    
        States(String state, double height) {
            this.state = state;
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }
    
    private double P = 0.75;
    private double I = 0;
    private double D = 0.01;

    DutyCycleOut pose;

    private TalonFX rightMotor;
    private TalonFX leftMotor;

    private final ProfiledPIDController controller;
    private final TrapezoidProfile.Constraints constraints;

    private States state;

    private final ElevatorFeedforward ff = 
        new ElevatorFeedforward(0, 0, 0);

    private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
    
    public Elevator() {
        rightMotor = new TalonFX(ElevatorConstants.ER_ID);
        leftMotor = new TalonFX(ElevatorConstants.EL_ID);
    
        constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
        controller = new ProfiledPIDController(P, I, D, constraints);
        controller.setTolerance(ElevatorConstants.TOLERANCE);
    
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; 
        slot0Configs.kV = 0.12; 
        slot0Configs.kA = 0.01; 
        slot0Configs.kP = 4.8; 
        slot0Configs.kI = 0; 
        slot0Configs.kD = 0.1; 

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 10; 
        motionMagicConfigs.MotionMagicAcceleration = 20; 
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        rightMotor.getConfigurator().apply(talonFXConfigs);
        leftMotor.getConfigurator().apply(talonFXConfigs);
    }
        
    public void setElevatorState(States state) {
        this.state = state;
        controller.setGoal(state.getHeight());
        System.out.println("new state" + state);
    }

    public void zeroEncoders() {
        rightMotor.setPosition(0.0);
        rightMotor.setPosition(0.0);
    }

    public double getHeight() {
        double elevatorpose = (rightMotor.getPosition().getValueAsDouble() + leftMotor.getPosition().getValueAsDouble()) / 2;
        elevatorpose = (elevatorpose * ElevatorConstants.GEAR_RATIO) * ElevatorConstants.LINEAR_DISTANCE_CONST;
        
        return elevatorpose;
    }

    public Command get() {
        return runOnce(() -> getHeight());
    }

    public void periodic() {
        SmartDashboard.putNumber("right motor pose", rightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("left motor pose", leftMotor.getPosition().getValueAsDouble());

        System.out.println(rightMotor.getPosition());
        System.out.println(leftMotor.getPosition());
    }
}
