// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;

public class Carriage extends SubsystemBase {
    public static double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    private TalonFX carriage;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private double last_pose;
    private double rotations;

    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    
    public Carriage() {
        carriage = new TalonFX(ElevatorConstants.CAR_ID);

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.01; 
        slot0Configs.kG = 0.785;
        slot0Configs.kV = 1.5; 
        slot0Configs.kA = 0.0025; 
        slot0Configs.kP = 12.5; 
        slot0Configs.kI = 0; 
        slot0Configs.kD = 0.1; 

        
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
       
        // var slot1Configs = talonFXConfigs.Slot1;
        // slot0Configs.kS = 0; 
        // slot0Configs.kG = 0.785;
        // slot0Configs.kV = 0.775; 
        // slot0Configs.kA = 0.0025; 
        // slot0Configs.kP = 21; 
        // slot0Configs.kI = 0; 
        // slot0Configs.kD = 0.01; 


        // var configs = talonFXConfigs.SoftwareLimitSwitch;
        // configs.withForwardSoftLimitEnable
        // (true);
        // configs.withForwardSoftLimitThreshold(-10);


        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; 
        motionMagicConfigs.MotionMagicAcceleration = 75; 
        motionMagicConfigs.MotionMagicJerk = 2500; 

        // var motionMagicConfigs2 = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 10;
        // motionMagicConfigs.MotionMagicAcceleration = 10; 
        // motionMagicConfigs.MotionMagicJerk = 400; 

        carriage.getConfigurator().apply(motionMagicConfigs);
        carriage.getConfigurator().apply(slot0Configs);
        //carriage.getConfigurator().apply(configs);
    
     //   carriage.getConfigurator().apply(talonFXConfigs);
        carriage.setNeutralMode(NeutralModeValue.Brake);

     //  setCarriagePosition(0);
    }

    public boolean isAtSetpoint(double goal) {
        return Math.abs(getCarriagePosition() - goal) <= 0.1;
    }

    // public boolean isAtSetpoint(double goal) {
    //     return 
    // }

    public double getCarriagePosition() {
       // double pose = carriage.get() * ElevatorConstants.C_GEAR_RATIO * ElevatorConstants.LINEAR_DISTANCE_CONST;
        double pose = carriage.getPosition().getValueAsDouble();
        return pose;
    }

    public void setCarriagePosition(double rotations) {
  //      final PositionVoltage request = new PositionVoltage(0).withSlot(0);

     //   carriage.setControl(request.withPosition(rotations));

       // final TrapezoidProfile m_profile = new TrapezoidProfile(
       // new TrapezoidProfile.Constraints( 120, 150));
        // Final target of 200 rot, 0 rps
       // TrapezoidProfile.State m_goal = new TrapezoidProfile.State(rotations, 0);
       // TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

        // create a position closed-loop request, voltage output, slot 0 configs
        //final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

        // calculate the next profile setpoint
       // m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        final MotionMagicVoltage m_request = new MotionMagicVoltage(rotations);

        // set target position to 100 rotations
        carriage.setControl(m_request.withPosition(-rotations));

        this.rotations = rotations;
    }

    public Command carriageCommand(double rotations) {
        return this.runOnce(() -> setCarriagePosition(rotations));
    }

    public void setPose(double pose) {
        carriage.setPosition(pose);
    }

    public void periodic() {
        SmartDashboard.putNumber("carriage pose", getCarriagePosition());

        SmartDashboard.putNumber("carriage rotor pose", carriage.getRotorPosition().getValueAsDouble());

       // carriage.setControl(m_request.withPosition(rotations));
    }
}
