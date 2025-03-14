package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
  
public class Climb extends SubsystemBase {
    private TalonFX Climbmotor;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State Setpoint;
    private final TrapezoidProfile.Constraints constaints;

    public Climb(){
        constaints = new TrapezoidProfile.Constraints(4.5, 4.5);
        goal = new TrapezoidProfile.State();
        Setpoint = new TrapezoidProfile.State();
        Climbmotor = new TalonFX(22);
    }

    public void setClimbSpeed(double speed){
        // Climbmotor.setVoltage(0.1);
        Climbmotor.set(speed);
        //Climbmotor.setPosition(180);
    }

    public Command climbspeedCommand(double speed){
        return this.runOnce(() -> setClimbSpeed(speed));
    }
    
    public Command stopmotor(double speed) {
        return this.runOnce(() -> stopmotor(speed));
    }

    public void stopMotor() {
        Climbmotor.stopMotor();
    }
           
    public Rotation2d getClimbAngle(){
        return Rotation2d.fromDegrees(180);
    }

    public TrapezoidProfile.State getSetpoint() {
        return Setpoint;
    }

    public TrapezoidProfile.State getgoal() {
        return goal;
    }

    public TrapezoidProfile.Constraints getconstraints() {
        return constaints;
    }
   
}
    
    
