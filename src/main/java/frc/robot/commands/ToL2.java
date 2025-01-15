package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ToL2 extends Command {
    private final Elevator elevator;
    private TrapezoidProfile current;
    private ElevatorFeedforward ff;
    private PIDController elevatorPID;

    public void initialize() {
        elevator.setGoal(ElevatorConstants.L2_HEIGHT.baseUnitMagnitude());
        System.out.println(elevator.getGoal());

        elevator.setSetpoint(
            new TrapezoidProfile.State(elevator.getElevatorPosition(), 0.0));
        System.out.println(elevator.getSetpoint());
    }

    public ToL2(Elevator elevator) {
        this.elevator = elevator;
        this.ff = new ElevatorFeedforward(
            ElevatorConstants.kS, 
            ElevatorConstants.kG, 
            ElevatorConstants.kV);
        this.elevatorPID = new PIDController(
            ElevatorConstants.P, 
            ElevatorConstants.I, 
            ElevatorConstants.D);
        addRequirements(elevator);
    }

    public void execute() {
        elevator.setPower(0);
        System.out.println("mr jackson rocks");
        current = new TrapezoidProfile(
            elevator.getConstraints());
        
        double pose = elevator.getElevatorPosition();
        // testing
        System.out.println("THIS IS POSE ONE LINE 41" + pose);

        TrapezoidProfile.State next = current.calculate(0.02, elevator.getSetpoint(), elevator.getGoal());
        // testing
        System.out.println("THIS IS THE NEXT ONE LINE 47" + next);

        double ff_power = ff.calculate(next.velocity) / 12;
        // testing
        System.out.println("FEED FORWARD POWER" + ff_power);

        elevator.setSetpoint(next);

        elevatorPID.setSetpoint(next.position);

        // testing
        System.out.println(elevator.getSetpoint());
        System.out.println(elevatorPID.getSetpoint());

        double power = elevatorPID.calculate(pose);

        double PIDFFpower = power + ff_power;
        // testing
        System.out.println("PIDFFpower" + PIDFFpower);

        elevator.setPower(PIDFFpower);
    }

    public void end(boolean interrupted) {
        elevator.setPower(0);
    }

    public boolean isFinished() {
        // if (Math.abs(elevator.getElevatorPosition() - ElevatorConstants.L2_HEIGHT.baseUnitMagnitude()) < ElevatorConstants.TOLERANCE) {
        //     return true;
        // } else {
        // return false;

        if (elevator.getElevatorPosition() < ElevatorConstants.L2_HEIGHT.magnitude()) {
            return false;
        } else {
            return true;
        }
    }}