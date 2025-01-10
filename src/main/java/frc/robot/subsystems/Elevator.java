package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    private double P = 0;
    private double I = 0;
    private double D = 0;

    TalonFX motor1 = new TalonFX(ElevatorConstants.E1_ID);
    TalonFX motor2 = new TalonFX(ElevatorConstants.E2_ID);

    private final TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration); 
    private final TrapezoidProfile profile =
        new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    
    
        private final ProfiledPIDController controller =
            new ProfiledPIDController(0, 0, 0, constraints);
        private final ElevatorFeedforward ff = 
            new ElevatorFeedforward(0, 0, 0);
        
    
        public Elevator() {}
    
        public void set_speed(double speed) {
            motor1.set(speed);
            motor2.set(speed);
        }
    
        public void setPID(double p, double i, double d) {
            p = P;
            i = I;
            d = D;
        }
    
        public void stop() {
            motor1.set(0);
            motor2.set(0);
        }
    
        public void resetEncoders() {
            motor1.setPosition(0);
            motor2.setPosition(0);
        }
    
        public void setSetpoint() {
        }

    public void periodic() {
        setpoint = profile.calculate(0.02, setpoint, goal);
    }
}
