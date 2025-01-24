// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ElevatorConstants;

// public class Elevator extends SubsystemBase {

//     DutyCycleOut pose;

//     private TalonFX rightMotor;
//     private TalonFX leftMotor;

//     private TrapezoidProfile current;

//     private final TrapezoidProfile.Constraints constraints;
//     private TrapezoidProfile.State goal;
//     private TrapezoidProfile.State setpoint;
        
//     public Elevator() {
//         rightMotor = new TalonFX(ElevatorConstants.ER_ID);
//         leftMotor = new TalonFX(ElevatorConstants.EL_ID);
        
//         constraints = new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
//         goal = new TrapezoidProfile.State();
//         setpoint = new TrapezoidProfile.State();
        
//         var talonFXConfigs = new TalonFXConfiguration();
    
//         // set slot 0 gains
//         var slot0Configs = talonFXConfigs.Slot0;
//         slot0Configs.kS = 0.25; 
//         slot0Configs.kV = 0.12; 
//         slot0Configs.kA = 0.01; 
//         slot0Configs.kP = ElevatorConstants.P; 
//         slot0Configs.kI = ElevatorConstants.I; 
//         slot0Configs.kD = ElevatorConstants.D; 
    
//         // set Motion Magic settings
//         var motionMagicConfigs = talonFXConfigs.MotionMagic;
//         motionMagicConfigs.MotionMagicCruiseVelocity = 10; 
//         motionMagicConfigs.MotionMagicAcceleration = 20; 
//         motionMagicConfigs.MotionMagicJerk = 1600;
    
//         rightMotor.getConfigurator().apply(talonFXConfigs);
//     }

//     public double getElevatorPosition() {
//         double elevatorpose = (rightMotor.getPosition().getValueAsDouble() + leftMotor.getPosition().getValueAsDouble()) / 2;
//         elevatorpose = (elevatorpose * ElevatorConstants.GEAR_RATIO) * ElevatorConstants.LINEAR_DISTANCE_CONST;

//         return elevatorpose;
//     }

//     public void zeroEncoders() {
//         rightMotor.setPosition(0);
//         leftMotor.setPosition(0);
//     }

//     // COMMANDS \\
//     public Command zero_command() {
//         return this.runOnce(() -> zeroEncoders());
//     }

//     public Command stop_command() {
//         return this.runOnce(() -> setPower(0));
//     }
    
//     // GETTERS \\
//     public TrapezoidProfile.State getSetpoint() {
//         return setpoint;
//     }

//     public TrapezoidProfile.State getGoal() {
//         return goal;
//     }

//     public TrapezoidProfile.Constraints getConstraints() {
//         return constraints;
//     }

//     public TrapezoidProfile getCurrent() {
//         return current;
//     }

//     // SETTERS \\
        
//     public void setPower(double power) {
//         rightMotor.set(power);
//         leftMotor.set(power);
//     }
            
//     public void setGoal(double goal_pose) {
//         goal = new TrapezoidProfile.State(goal_pose, 0);
//     }

//     public void setSetpoint(TrapezoidProfile.State setpoint) {
//         this.setpoint = setpoint; 
//     }

//     public void periodic() {
//         SmartDashboard.putNumber("right motor pose", rightMotor.getPosition().getValueAsDouble());
//         SmartDashboard.putNumber("left motor pose", leftMotor.getPosition().getValueAsDouble());
//         SmartDashboard.putNumber("elevator pose", getElevatorPosition());
//     }
// }
