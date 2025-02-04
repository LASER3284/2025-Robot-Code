package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
           public class Roller extends SubsystemBase {
            private TalonFX Rollermotor;
            private TalonFX Rollermotor2;
            public Roller(){
            Rollermotor = new TalonFX(12);
            Rollermotor2 = new TalonFX(11);
        }
          public void setRollerSpeed(double speed){
            Rollermotor.set(speed);
            Rollermotor2.set(speed);
        }
        public Command RollerspeedCommand(double speed){
            return this.runOnce(() -> setRollerSpeed(speed));
        }
        public Command stopmotor(double speed)
        {
            return this.runOnce(() -> stopmotor(speed));
        }
        public void stopMotor() {
            Rollermotor.stopMotor();
            Rollermotor2.stopMotor();
        }
    }
           
        