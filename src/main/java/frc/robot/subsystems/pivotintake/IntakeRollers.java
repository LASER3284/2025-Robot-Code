package frc.robot.subsystems.pivotintake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private static IntakeRollers instance;

    private TalonFX Intakemotor;

    public IntakeRollers() {
        Intakemotor = new TalonFX(16);
    }

    public static IntakeRollers getInstance() {
        if (instance == null) {
            instance = new IntakeRollers();
        }
        return instance;
    }

    public void setMotorSpeed(double speed){
        Intakemotor.set(speed);
    }

    public void stop(){
        Intakemotor.stopMotor();
    }

    public void periodic() {}

    public Command setMotorSpeed_command(double speed){
        return this.runOnce(() -> Intakemotor.set(speed));
    }

    public Command setIntake(){
        return this.runOnce(() -> setIntake());
    }

    
}
