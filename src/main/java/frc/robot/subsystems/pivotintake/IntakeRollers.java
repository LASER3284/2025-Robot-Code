package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private TalonFX Intakemotor;

    public IntakeRollers() {
        Intakemotor = new TalonFX(16);
    }

    public void setMotorSpeed(double speed){
        Intakemotor.set(speed);
    }

    public void stop(){
        Intakemotor.stopMotor();
    }
    
    @Override
    public void periodic() {}

    public void logOutputs(){

    }

    public Command setMotorSpeed_command(double speed){
        return this.runOnce(() -> Intakemotor.set(speed));
    }

    public Command setIntake(){
        return this.runOnce(() -> setIntake());
    }

    
}
