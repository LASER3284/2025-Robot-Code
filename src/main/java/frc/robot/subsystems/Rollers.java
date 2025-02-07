package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class Rollers extends SubsystemBase{

    private TalonFX coral_motor;
    private TalonFX algae_motor;

    public Rollers(){
        coral_motor = new TalonFX(RollerConstants.CORAL_ROLLER_ID);
        algae_motor = new TalonFX(RollerConstants.ALGAE_ROLLER_ID);
    }

    public void algae_roller_on(double speed){
        algae_motor.set(speed);
    }

    public void coral_roller_on(double speed){
        coral_motor.set(speed);
    }
    
    public Command algae_roller_on_command(double speed){
        return this.runOnce(() -> algae_roller_on(speed));
    }

    public Command coral_roller_on_command(double speed){
        return this.runOnce(() -> coral_roller_on(speed));
    }
}
