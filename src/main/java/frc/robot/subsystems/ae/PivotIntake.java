package frc.robot.subsystems.ae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotIntakeConstants;

public class PivotIntake {
    private TalonFX pivot_motor;
    private SparkMax roller_motor;
    private Encoder thru_bore;

    private TrapezoidProfile current;

    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public PivotIntake () {
        pivot_motor = new TalonFX(PivotIntakeConstants.CPIVOT_ID);
        roller_motor = new SparkMax(PivotIntakeConstants.CROLLER_ID, MotorType.kBrushless);
        thru_bore = new Encoder(0, 1);

        constraints = new TrapezoidProfile.Constraints(PivotIntakeConstants.maxVelocity, PivotIntakeConstants.maxAcceleration);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        current = new TrapezoidProfile(constraints);

        var talonFXConfigs = new TalonFXConfiguration();
    
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; 
        slot0Configs.kV = 0.12; 
        slot0Configs.kA = 0.01; 
        slot0Configs.kP = PivotIntakeConstants.P; 
        slot0Configs.kI = PivotIntakeConstants.I; 
        slot0Configs.kD = PivotIntakeConstants.D; 
    
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 10; 
        motionMagicConfigs.MotionMagicAcceleration = 20; 
        motionMagicConfigs.MotionMagicJerk = 1600;
    
        pivot_motor.getConfigurator().apply(talonFXConfigs);
    }

    public double getPivotPosition() {
        double pivotpose = pivot_motor.getPosition().getValueAsDouble();
        pivotpose = (pivotpose * PivotIntakeConstants.GEAR_RATIO);

        return pivotpose;
    }

    public double getEncoder() {
        return thru_bore.getDistance();
    }

    public void setRollerSpeed(double speed) {
        roller_motor.set(speed);
    }

    public void periodic() {
        SmartDashboard.putNumber("pivot pose", getPivotPosition());
        SmartDashboard.putNumber("encoder pose", getEncoder());
    }

}
