package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ToHome extends Command {
    private Elevator elevator;

    private TrapezoidProfile current_ele;
    private TrapezoidProfile current_car;

    private ElevatorFeedforward ff;
    private PIDController elevatorPID;
    private Distance eheight;
    private Distance cheight;

    public void initialize() {
        elevator.setEGoal(eheight.magnitude());
        elevator.setESetpoint(
            new TrapezoidProfile.State(elevator.getElevatorPosition(), 0.0));

        elevator.setCGoal(cheight.magnitude());
        elevator.setCSetpoint(
            new TrapezoidProfile.State(elevator.getCarriagePosition(), 0.0));
    }

    public ToHome(Elevator elevator, Distance eheight, Distance cheight) {
        this.elevator = elevator;
        this.eheight = eheight;
        this.cheight = cheight;

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
        elevator.setElevatorPower(0);
        elevator.setCarriagePower(0);


        current_ele = new TrapezoidProfile(
            elevator.getConstraints("elevator"));
        double epose = elevator.getElevatorPosition();
        TrapezoidProfile.State enext = current_ele.calculate(0.02, elevator.getSetpoint("elevator"), elevator.getGoal("elevator"));
        double eff_power = ff.calculate(enext.velocity) / 12;
        elevator.setESetpoint(enext);
        elevatorPID.setSetpoint(enext.position);
        double epower = elevatorPID.calculate(epose);
        double etotalpower = epower + eff_power;

        current_car = new TrapezoidProfile(
            elevator.getConstraints("elevator"));
        double cpose = elevator.getElevatorPosition();
        TrapezoidProfile.State cnext = current_car.calculate(0.02, elevator.getSetpoint("elevator"), elevator.getGoal("elevator"));
        double cff_power = ff.calculate(cnext.velocity) / 12;
        elevator.setESetpoint(cnext);
        elevatorPID.setSetpoint(cnext.position);
        double power = elevatorPID.calculate(cpose);
        double ctotalpower = power + cff_power;



        elevator.setElevatorPower(etotalpower);
        elevator.setCarriagePower(ctotalpower);
    }

    public void end(boolean interrupted) {
        elevator.setElevatorPower(0);
        elevator.setCarriagePower(0);
    }

    public boolean isFinished() {
        return elevator.getLimit(); 
    }
}
