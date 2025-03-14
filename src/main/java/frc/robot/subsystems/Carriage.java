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
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;

public class Carriage extends SubsystemBase {
    private static Carriage instance;

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

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; 
        motionMagicConfigs.MotionMagicAcceleration = 75; 
        motionMagicConfigs.MotionMagicJerk = 2500; 

        carriage.getConfigurator().apply(motionMagicConfigs);
        carriage.getConfigurator().apply(slot0Configs);

        carriage.setNeutralMode(NeutralModeValue.Brake);
    }

    public static Carriage getInstance() {
        if (instance == null) {
            instance = new Carriage();
        }
        return instance;
    }

    public boolean isAtSetpoint(double goal) {
        return Math.abs(getCarriagePosition() - goal) <= CarriageConstants.TOLERANCE;
    }

    public double getCarriagePosition() {
        double pose = carriage.getPosition().getValueAsDouble();
        return pose;
    }

    public void setCarriagePosition(double rotations) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(rotations);

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
    }
}
