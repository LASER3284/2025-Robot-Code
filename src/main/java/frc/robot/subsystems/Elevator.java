// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;

public class Elevator extends SubsystemBase {
    public static double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    private TalonFX rightElevator;
    private TalonFX leftElevator;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private double last_pose;
    private double rotations;

    VoltageOut request = new VoltageOut(0);
    MotionMagicVoltage m_magicRequest = new MotionMagicVoltage(0);
    
    public Elevator() {
        rightElevator = new TalonFX(41);
        leftElevator = new TalonFX(42);

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0; 
        slot0Configs.kG = -25;
        slot0Configs.kV = 0.6; 
        slot0Configs.kA = 0.0025; 
        slot0Configs.kP = 1; 
        slot0Configs.kI = 0; 
        slot0Configs.kD = 0.01; 
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 50; 
        motionMagicConfigs.MotionMagicAcceleration = 90; 
        motionMagicConfigs.MotionMagicJerk = 200; 

        rightElevator.getConfigurator().apply(motionMagicConfigs);
        leftElevator.getConfigurator().apply(slot0Configs);

        leftElevator.setControl(new Follower(41, true));
    //    carriage.getConfigurator().apply(talonFXConfigs);
        rightElevator.setNeutralMode(NeutralModeValue.Brake);

     //  setCarriagePosition(0);
    }

    public double getElevatorPosition() {
        double pose = rightElevator.get() * ElevatorConstants.GEAR_RATIO * ElevatorConstants.LINEAR_DISTANCE_CONST;

        return pose;
    }

    public void setElevatorPosition(double rotations) {
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
        rightElevator.setControl(m_request.withPosition(-rotations));

        this.rotations = rotations;
    }

    public Command elevatorCommand(double rotations) {
        return this.runOnce(() -> setElevatorPosition(rotations));
    }

    public boolean isAtHome(double goal) {
        return Math.abs(getElevatorPosition() - goal) <= 0.1;
    }

    public void periodic() {
        SmartDashboard.putNumber("right elevator pose", rightElevator.get());
        SmartDashboard.putNumber("left elevator pose", leftElevator.get());
    }
}
