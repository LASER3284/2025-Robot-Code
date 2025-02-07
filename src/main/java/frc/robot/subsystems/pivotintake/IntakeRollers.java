package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.controls.ControlRequest;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private SparkMax Intakemotor;

    public IntakeRollers() {
        Intakemotor = new SparkMax(0, MotorType.kBrushless);
    }

    public void setMotorSpeed(double speed){
        Intakemotor.set(speed);
    }

    public void setIntake(ControlRequest control){
        Intakemotor.setControlFramePeriodMs(0);
    }

    public void stop(){
        Intakemotor.stopMotor();
    }
    
    @Override
    public void periodic() {}

    public void logOutputs(){

    }

    public Command setMotorSpeed(){
        return this.runOnce(() -> setMotorSpeed());
    }

    public Command setIntake(){
        return this.runOnce(() -> setIntake());
    }

    
}
