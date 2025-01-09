package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;

public class Elevator {

    double kP;
    double kI;
    double kD;

    public enum PIDmodes {
        position,
        velocity,
        with_algae,
        without_algae
    }

    TalonFX motor1 = new TalonFX(Constants.ElevatorConstants.E1_ID);
    TalonFX motor2 = new TalonFX(Constants.ElevatorConstants.E2_ID);

    PIDController pid = new PIDController(kP, kI, kD);

    public Elevator() {}

    public void set_speed(double s1, double s2) {
        motor1.set(s1);
        motor2.set(s2);
    }

    public void stop() {
        motor1.set(0);
        motor2.set(0);
    }
    // gonna need a trapezoidal pid

    // set points and goals and such
}
