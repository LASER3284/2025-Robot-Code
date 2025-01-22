package frc.robot.subsystems.ae;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private SparkFlex rackmotor;
    private TalonFX rollermotor;
    private TrapezoidProfile current;
    private PIDController AlgaePID;
    private ElevatorFeedforward Feed;
    private final TrapezoidProfile.Constraints constaints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State Setpoint;

// Extend levels
   enum state {extended,retracted,hardLimit}
   
    public AlgaeIntake() {
            rollermotor = new TalonFX(2);
            rackmotor = new SparkFlex(1, MotorType.kBrushless);          
            constaints = new TrapezoidProfile.Constraints(0.2,0.2);
            goal = new TrapezoidProfile.State();
            Setpoint = new TrapezoidProfile.State();
            this.Feed = new ElevatorFeedforward(
                0.1,
                0.2,
                0.3);
            this.AlgaePID = new PIDController(
                   0.1,
                   0, 
                   0.001);
                   // PID numbers
    }
    public double getAlgaePosition() {
        double Algaepose = rackmotor.getExternalEncoder().getPosition();
        Algaepose = (Algaepose * (36/15)) * Math.PI;
    // Motor Position
        return Algaepose;
    }
    public void encoders() {
        rackmotor.getEncoder().getPosition();
    }

    public void setRackSpeed(double speed) {
        rackmotor.set(speed);
    }

    public void setRollerSpeed(double speed) {
        rollermotor.set(speed);
    }
    public void extend(double distance) {
        setgoal(distance);
        setsetpoint(
        new TrapezoidProfile.State(getAlgaePosition(),0));
        System.out.println("Distance: " + distance);
        setRackSpeed(0);
        current = new TrapezoidProfile(
        getconstraints());
        double pose = getAlgaePosition();
        TrapezoidProfile.State next = current.calculate(0.02, getSetpoint(), getgoal());
        double Feeds = Feed.calculate(next.velocity) / 12;
        double Feed_power = AlgaePID.calculate(pose);
        double Feed = Feeds + Feed_power;
        setRackSpeed(Feed);
    }

        public Command extend_command(double distance)
        {
            return this.runOnce(() -> extend(distance));
        }

        public Command stopmotor()
        {
            return this.runOnce(() -> stopmotor());
        }

        //GETTERS
        public TrapezoidProfile.State getSetpoint() {
            return Setpoint;
        }
        public TrapezoidProfile.State getgoal() {
            return goal;
        }
        public TrapezoidProfile.Constraints getconstraints() {
            return constaints;
        }
        public PIDController getAlgaePID() {
            return AlgaePID;
        }
        public ElevatorFeedforward getFeed() {
            return Feed;
        }
        public TrapezoidProfile getcurrent() {
            return current;
        }

        //SETTERS
        public void setgoal(double goal_pose) {
            goal = new TrapezoidProfile.State(goal_pose,0);
        }
        public void setsetpoint(TrapezoidProfile.State setpoint) {
            this.Setpoint = setpoint;
        }
    public void stopMotor() {
        rollermotor.stopMotor();
        rackmotor.stopMotor();
    }
    public void periodic() {
        //SmartDashboard.putNumber("distance", encoder.getDistance());
        SmartDashboard.putNumber("Distance", getAlgaePosition());
        SmartDashboard.putNumber("motor dist", rackmotor.getEncoder().getPosition());
    }   


}