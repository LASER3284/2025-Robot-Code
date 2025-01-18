package frc.robot.subsystems.ae;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotIntakeConstants;

public class PivotIntake {
    private TalonFX pivot_motor;
    private SparkMax roller_motor;
    private Encoder thru_bore;

    public PivotIntake () {
        pivot_motor = new TalonFX(PivotIntakeConstants.CPIVOT_ID);
        roller_motor = new SparkMax(PivotIntakeConstants.CROLLER_ID, MotorType.kBrushless);
        thru_bore = new Encoder(0, 1);
    }

    public double getPivotPosition() {
        double pivotpose = pivot_motor.getPosition().getValueAsDouble();
        pivotpose = (pivotpose * PivotIntakeConstants.GEAR_RATIO) * PivotIntakeConstants.LINEAR_DISTANCE_CONST;

        return pivotpose;
    }
}
