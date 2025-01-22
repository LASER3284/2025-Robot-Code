package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class pivot extends SubsystemBase {
/* ------------------------------------------------------------------------------------
    //idk if im actually going to be using these trapezoid profile thingys so im commenting it out for now!! :D
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile profile;
    private PIDController pid;
*///------------------------------------------------------------------------------------

public TalonFX pivotMaster = new TalonFX(PivotConstants.pivotMotorID);

private double P = 0;
private double I = 0;
private double D = 0;


double pivotPos;

    private final TalonFX pivotMotor = new TalonFX(1);            

     public pivot()
     {
        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration pivot_config = new TalonFXConfiguration();
        pivotMotor.getConfigurator().apply(pivot_config);


    pivot_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivot_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 8.2;
    pivot_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivot_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.1;
    pivot_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotMotor.getConfigurator().apply(pivot_config);
     }

     public void setPID(double p, double i, double d) {
        p = P;
        i = I;
        d = D;
    }

//switch statements so it goes to that angle
     public void gotoAngle(double setPoint)
     {
        //*hopeful motion_magic music plays*
        MotionMagicConfigs pivot_magic = new MotionMagicConfigs();
            pivot_magic.MotionMagicCruiseVelocity = 90;
            pivot_magic.MotionMagicAcceleration = 35;

        pivotMotor.getConfigurator().apply(pivot_magic);
        final MotionMagicVoltage magic_request = new MotionMagicVoltage(0).withSlot(0);
        
        pivotMotor.setControl(magic_request.withPosition(setPoint));
     }


      public double updatePivotPos()
      {
        pivotPos = pivotMotor.getPosition().getValueAsDouble();
        pivotPos = (pivotPos * PivotConstants.GEAR_RATIO);
        return pivotPos;
      }
    
     
      public double returnPivotPos()
      {
        
        updatePivotPos();
        return pivotPos;
      }
      
     public void goToSetPoint(double setPoint)
     {
        var slot0Configs = new Slot0Configs();
          slot0Configs.kG = PivotConstants.G;
          slot0Configs.kP = PivotConstants.P;
          slot0Configs.kI = 0;
          slot0Configs.kD = PivotConstants.D;
          slot0Configs.kV = PivotConstants.V;
          slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

          pivotMotor.getConfigurator().apply(slot0Configs);
          MotionMagicConfigs pivot_magic = new MotionMagicConfigs();
          pivot_magic.MotionMagicCruiseVelocity = 90;
          pivot_magic.MotionMagicAcceleration = 35;
          pivotMotor.getConfigurator().apply(pivot_magic);
          final MotionMagicVoltage magic_request = new MotionMagicVoltage(0).withSlot(0);
          
          pivotMotor.setControl(magic_request.withPosition(setPoint));
     }


//sets position to 0. whatever that is?
     public void setZero()
     {
        pivotMotor.setPosition(0);
     } 

     public void resetPivot()
      {
     final VoltageOut m_request = new VoltageOut(0);
    pivotMotor.setControl(m_request.withOutput(0));
      }

     @Override
        public void periodic()
        {
            SmartDashboard.putNumber("Raw Pivot Position: ", returnPivotPos());
        }
}
