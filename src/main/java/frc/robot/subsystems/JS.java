package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.JSConstants;
import frc.robot.commands.PivotToAngle;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class JS extends SubsystemBase {
    private static JS instance;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private Rollers rollers;

    private TrapezoidProfile current;
    private TalonFX pivotMotor;
    private DutyCycleEncoder thru_bore = new DutyCycleEncoder(JSConstants.port);

    private ArmFeedforward ff;

    private TrapezoidProfile current_js;

    private PIDController pid;

    public double current_pose = 0;
    private double last_goal;

    public JS() {
        pivotMotor = new TalonFX(JSConstants.JS_ID);
        //thru_bore = new DutyCycleEncoder(3);


        constraints = new TrapezoidProfile.Constraints(800, 1600);
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        this.ff = new ArmFeedforward(
            JSConstants.kS, 
            JSConstants.kG,
            JSConstants.kV, 
            JSConstants.kA
        );

        pid = new PIDController(
            JSConstants.kP,
            JSConstants.kI,
            JSConstants.kD);

        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        last_goal = JSConstants.STOW;
    }

    public static JS getInstance() {
        if (instance == null) {
            instance = new JS();
        }
        return instance;
    }

    public void initDefaultCommand() {
        setDefaultCommand(new PivotToAngle(this, rollers, last_goal));
    }

    public double getPivotPosition() {
        double pivotPose = thru_bore.get();
        return pivotPose;
    }

    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    public TrapezoidProfile.State getGoal() {
        return goal;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public TrapezoidProfile getCurrent() {
        return current;
    }

    public PIDController getPID() {
        return pid;
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public Command stop_command() {
        return this.runOnce(() -> setPower(0));
    }

    public Command setSpeed_command(double speed) {
        return this.runOnce(() -> setSpeed(speed));
    }

    public void setPower(double power) {
        pivotMotor.set(power);
    }

    public void setGoal(double current) {
        goal = new TrapezoidProfile.State(current, 0.1);
    }

    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint; 
    }

    public boolean isSafeGroundIn() {
        return thru_bore.get() < 0.75 && thru_bore.get() > 0.55;
    }

    public boolean isAtSetpoint(double angle) {
        return (Math.abs(thru_bore.get()) - angle) < 0.05;
    }

    public void calculateJSPose(double angle) {
        setGoal(angle);
        setSetpoint(
            new TrapezoidProfile.State(getPivotPosition(), 0.0));

        setLastGoal(angle);

        current_js = new TrapezoidProfile(
            getConstraints());
        double cpose = getPivotPosition();
        TrapezoidProfile.State cnext = current_js.calculate(0.02, getSetpoint(), getGoal());
         double cff_power = ff.calculate(cnext.position, cnext.velocity);
        setSetpoint(cnext);
        getPID().setSetpoint(cnext.position);
        double cpower = getPID().calculate(cpose);
        double ctotalpower = cpower + cff_power;

        SmartDashboard.putNumber("ctotalpower", ctotalpower);

        pivotMotor.set(ctotalpower);
    }

    public void setPose(double angle) {
        current_pose = angle;
    }

    public Command calcCommand(double angle) {
        return this.run(() -> calculateJSPose(angle));
    }

    public Command setPoseCommand(double angle) {
        return this.run(() -> setPose(angle));
    }

    public void setLastGoal(double angle) {
        this.last_goal = angle;
    }

    
    public double getLastGoal() {
        return last_goal;
    }

    public Command setGoalPose() {
        return this.runOnce(() -> setLastGoal(last_goal));
    }

    public Command setGoalPose(double value) {
        return this.runOnce(() -> setLastGoal(value));
    }

    public void periodic() {
        SmartDashboard.putNumber("encoder pose", thru_bore.get());
        SmartDashboard.putNumber("js pose", getPivotPosition());//getPivotPosition());
        SmartDashboard.putNumber("current pose", current_pose);
        SmartDashboard.putNumber("last goal", getLastGoal());
        initDefaultCommand();
    }
}