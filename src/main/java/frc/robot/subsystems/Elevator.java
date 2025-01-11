package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private double P = 0;
    private double I = 0;
    private double D = 0;

    private TalonFX rightMotor;
    private TalonFX leftMotor;

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
        
    
    public Elevator() {
        rightMotor = new TalonFX(ElevatorConstants.ER_ID);
        leftMotor = new TalonFX(ElevatorConstants.EL_ID);
    }
    
    public void set_speed(double speed) {
        rightMotor.set(speed);
        leftMotor.set(speed);
    }
    
    public void setPID(double p, double i, double d) {
        p = P;
        i = I;
        d = D;
    }
    
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }
    
    public void zeroEncoders() {
        rightMotor.setPosition(0.0);
        leftMotor.setPosition(0.0);
    }

    public void setElevator(ControlRequest control) {
        rightMotor.setControl(control);
        leftMotor.setControl(control);
    }
    
    public void setSetpoint(double new_pose, double new_vel) {
        profile.calculate(0.02, new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(new_pose, new_vel));
    }

    public void setNeutralMode(NeutralModeValue mode) {
        rightMotor.setNeutralMode(mode);
        leftMotor.setNeutralMode(mode);
    }

    public Command zero_enc() {
        return run(() -> zeroEncoders());
    }

    public Command speed(double speed) {
        return run(() -> set_speed(speed));
    }

    public void periodic() {
        setpoint = profile.calculate(0.02, setpoint, goal);
        SmartDashboard.putNumber("right motor pose", rightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("left motor pose", leftMotor.getPosition().getValueAsDouble());
    }
}
