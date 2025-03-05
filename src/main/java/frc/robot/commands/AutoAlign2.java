package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AutoAlign2 extends Command {
    private Drivetrain drivetrain;
    private LimelightHelpers ll_tags;
    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds;
    private double targetAngle;
    private double offset;
    private String name;

    private static final double limelightHeight = 5.3125;  // Example height in inches
    private static final double targetHeight = 12;  

    
    public void initialize() {}

    public AutoAlign2(Drivetrain drivetrain, LimelightHelpers ll_tags, double offset, String name) {
        this.offset = offset;
        this.drivetrain = drivetrain;
        this.ll_tags = ll_tags;
        this.name = name;
        pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        //drivetrain.setControl(null);
    }

    public void execute() {

        double tx = LimelightHelpers.getTX(name); // Horizontal offset
        double ty = LimelightHelpers.getTY(name); // Vertical offset (used for distance)

        // Calculate robot heading (rotation)
        double targetAngle = tx; // Rotate to align with target's horizontal offset

        // Calculate robot distance to target (can be based on ty or another method)
        double targetDistance = calculateDistance(ty); // Use ty to calculate distance
        SmartDashboard.putNumber("targetDistance", targetDistance);

        // Create and apply the swerve control request (adjust as needed)
        // Set the robot's speed to drive toward the target
        drivetrain.setControl(
            pathApplyRobotSpeeds.withSpeeds(
                new ChassisSpeeds(targetDistance * 0.01, (targetAngle + offset) * 0.01 , 0) // Adjust speed for your robot
            )
        );
    }

    public void end(boolean interruped) {
        drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTX(name)) < 0.5 && Math.abs(LimelightHelpers.getTY(name)) < 0.5; // Thresholds for alignment
    }

    private double calculateDistance(double ty) {
        // Use trigonometry to calculate the distance to the target based on ty (vertical angle)
        // You can adjust this based on your specific robot setup
        double dist = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(ty));
        SmartDashboard.putNumber("calc distance", dist);
        return dist;
    }
}
