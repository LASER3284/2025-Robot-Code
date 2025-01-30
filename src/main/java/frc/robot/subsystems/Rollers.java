package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;

public class Rollers {
    private TalonFX scorer_motor;
    private TalonFX algae_motor;

    public Rollers(){
        scorer_motor = new TalonFX(30);
        algae_motor = new TalonFX(31);
    }

    public void Aroller_on(double speed){
        algae_motor.set(speed);

    }

    public void Croller_on(double speed){
        scorer_motor.set(speed);
    }
}
