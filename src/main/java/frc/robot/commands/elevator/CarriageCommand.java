package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage;

public class CarriageCommand extends Command {
    private double rotations;

    private Carriage carriage = new Carriage();

    public CarriageCommand(double rotations) {
        this.rotations = rotations;
    }

    public void initialize() {
        //this.rotations = rotations;
    }

    public void execute() {
       
        carriage.setCarriagePosition(rotations);
    }

    public void end(boolean interrupted) {
    }
    
    public boolean isFinished() {
        

        return Math.abs(Math.abs(carriage.getCarriagePosition()) - Math.abs(rotations)) < 0.5;

    }
}
