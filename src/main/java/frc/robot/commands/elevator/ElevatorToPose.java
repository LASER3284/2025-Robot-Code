package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToPose extends Command {

    private Elevator elevator;
    private double pose;

    public void initialize() {

    }

    public ElevatorToPose(Elevator elevator, double pose) {
        this.elevator = elevator;
        this.pose = pose;
    }

    public void execute() {
        elevator.goToSetPoint(pose);
    }

    public void end(boolean interrupted) {
        elevator.stopElevatorMotor();
    }

    public boolean isFinished() {
        return elevator.atSetpoint(pose);
    }
}
