package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {
    private double rotations;

    private Elevator elevator = new Elevator();

    public void initialize() {}

    public ElevatorCommand(double rotations) {}

    public void execute() {
        elevator.setElevatorPosition(rotations);
    }

    public void end(boolean interrupted) {
       elevator.setElevatorSpeed(0);
    }
    
    public boolean isFinished() {
        return Math.abs(Math.abs(elevator.getElevatorPosition()) - rotations) < 0.1;
    }
}
