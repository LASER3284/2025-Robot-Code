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


    private double saved_tx = 0;
    private double saved_ty = 0;

    private static final double limelightHeight = 8.375;  // Example height in inches
    private static final double targetHeight = 12;  

    
    public void initialize() {
        saved_tx = LimelightHelpers.getTX(name);
        saved_ty = LimelightHelpers.getTY(name);

    }

    public AutoAlign2(Drivetrain drivetrain, LimelightHelpers ll_tags, double offset, String name) {
        this.offset = offset;
        this.drivetrain = drivetrain;
        this.ll_tags = ll_tags;
        this.name = name;
        pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        //drivetrain.setControl(null);
    }

    public void execute() {

        // double tx = LimelightHelpers.getTX(name); // Horizontal offset
        // double ty = LimelightHelpers.getTY(name); // Vertical offset (used for distance)

        // Calculate robot heading (rotation)
        double targetAngle = saved_tx; // Rotate to align with target's horizontal offset

        // Calculate robot distance to target (can be based on ty or another method)
        double targetDistanceX = calculateDistanceTY(saved_tx); // Use ty to calculate distance
        double targetDistanceY = calculateDistanceTX(saved_tx, saved_ty);
        SmartDashboard.putNumber("targetDistance", targetDistanceX);

        // Create and apply the swerve control request (adjust as needed)
        // Set the robot's speed to drive toward the target
        drivetrain.setControl(
            pathApplyRobotSpeeds.withSpeeds(
                new ChassisSpeeds(targetDistanceX * 0.01, (targetDistanceY + offset) * 0.01 , 0) // Adjust speed for your robot
            )
        );
    }

    public void end(boolean interruped) {
        drivetrain.setControl(pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds((saved_tx - LimelightHelpers.getTX(name)) * 0.001, (saved_ty - LimelightHelpers.getTY(name)) * 0.001, 0)));
    }

    public boolean isFinished() {
        return Math.abs(LimelightHelpers.getTX(name)) < 0.1 && Math.abs(LimelightHelpers.getTY(name)) < 0.1; // Thresholds for alignment
    }

    private double calculateDistanceTY(double ty) {
        // Use trigonometry to calculate the distance to the target based on ty (vertical angle)
        // You can adjust this based on your specific robot setup
        double dist = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(ty));
        SmartDashboard.putNumber("calc distance", dist);
        return dist;
    }

    private double calculateDistanceTX(double tx, double ty) {
        double dist = calculateDistanceTY(ty) * Math.tan(Math.toRadians(tx));
        return dist;
    }
}
