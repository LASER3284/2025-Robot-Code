package frc.robot.subsystems.ae;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private SparkFlex rackmotor;
    private SparkMax rollermotor;
    private DigitalInput limit;

    private TrapezoidProfile current;
    private PIDController AlgaePID;
    private ElevatorFeedforward Feed;
    private final TrapezoidProfile.Constraints constaints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State Setpoint;
   
    public AlgaeIntake() {
            rollermotor = new SparkMax(2, MotorType.kBrushless);
            rackmotor = new SparkFlex(1, MotorType.kBrushless);
            limit = new DigitalInput(9);
            
            constaints = new TrapezoidProfile.Constraints(4.5, 4.5);
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
        double algaepose = rackmotor.getEncoder().getPosition();
        SmartDashboard.putNumber("og pose", algaepose);
        algaepose = (algaepose * (36/15)) * Math.PI;
    // Motor Position
        return algaepose;
    }
    public void encoders(double pose) {
        rackmotor.getEncoder();
    }
    public void setRackSpeed(double speed) {
        rackmotor.set(speed);
        //motor speed
    }

    public void zeroEncoder() {
        rackmotor.getEncoder().setPosition(0);
    }

    public void setRollerSpeed(double speed) {
        rollermotor.set(speed);
    }

    public Command stopmotor()
    {
        return this.runOnce(() -> stopmotor());
    }

    public Command zero_command() {
        return this.runOnce(() -> zeroEncoder());
    }

    public Command rollerSpeed_Command(double speed) {
        return this.runOnce(() -> setRollerSpeed(speed));
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

    public boolean getlimit() {
        return limit.get();
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
        SmartDashboard.putNumber("getalgae pose", getAlgaePosition());
        SmartDashboard.putNumber("motor dist", rackmotor.getEncoder().getPosition());
    }   


}