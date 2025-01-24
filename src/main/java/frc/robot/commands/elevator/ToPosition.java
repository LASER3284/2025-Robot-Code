// package frc.robot.commands.elevator;

// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.subsystems.Elevator;

// public class ToPosition extends Command {
//     private final Elevator elevator;
//     private TrapezoidProfile current;
//     private ElevatorFeedforward ff;
//     private PIDController elevatorPID;
//     private Distance height;

//     public void initialize() {
//         elevator.setGoal(height.magnitude());

//         elevator.setSetpoint(
//             new TrapezoidProfile.State(elevator.getElevatorPosition(), 0.0));
//     }

//     public ToPosition(Elevator elevator, Distance height) {
//         this.elevator = elevator;
//         this.height = height;

//         this.ff = new ElevatorFeedforward(
//             ElevatorConstants.kS, 
//             ElevatorConstants.kG, 
//             ElevatorConstants.kV);
//         this.elevatorPID = new PIDController(
//             ElevatorConstants.P, 
//             ElevatorConstants.I, 
//             ElevatorConstants.D);
//         addRequirements(elevator);
//     }

//     public void execute() {
//         elevator.setPower(0);
//         current = new TrapezoidProfile(
//             elevator.getConstraints());
//         double pose = elevator.getElevatorPosition();
//         TrapezoidProfile.State next = current.calculate(0.02, elevator.getSetpoint(), elevator.getGoal());
//         double ff_power = ff.calculate(next.velocity) / 12;
//         elevator.setSetpoint(next);

//         elevatorPID.setSetpoint(next.position);
//         double power = elevatorPID.calculate(pose);
//         double PIDFFpower = power + ff_power;
//         elevator.setPower(PIDFFpower);
//     }

//     public void end(boolean interrupted) {
//         elevator.setPower(0);
//     }

//     public boolean isFinished() {
//         return Math.abs(elevator.getElevatorPosition() - height.magnitude()) < ElevatorConstants.TOLERANCE;
//     }
// }
