package frc.robot.subsystems;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Rollers {
    Spark motor = new Spark(0);
    public void setspeed(){
        motor.set(.5);
    }
    
   
}
