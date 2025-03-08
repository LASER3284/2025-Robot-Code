package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AutoAlign extends Command{
    private Drivetrain drivetrain;
    private LimelightHelpers ll;

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds;

    private final double MaxSpeed;
    private final double MaxAngularRate;

    private final double targetDistance; // Desired distance from the tag
    private final double targetAngle; // Desired angle relative to the tag

    private static final double kP_aim = 0.075; // Proportional gain for aiming
    private static final double kP_range = -0.1; // Proportional gain for ranging
    private static final double kP_horizontal = 0.05; // Reduced proportional gain for horizontal movement
    private static final double distanceTolerance = 0.1; // Tolerance for distance in meters
    private static final double angleTolerance = 1.0; // Degrees tolerance for alignment

    private double lastValidTargetTX = 0.0;
    private double lastValidTargetTY = 0.0;

    private double lastValidTargetAngle = 0.0;
    private double lastHorizontalAdjust = 0.0; // Last horizontal adjustment for smoothing

    private final Timer lostDetectionTimer = new Timer();
    private static final double lostDetectionTimeout = 0.5; // 0.5 seconds timeout for lost detection

    private String name;


    public void initialize() {}

    public AutoAlign(Drivetrain drivetrain, LimelightHelpers ll, double targetDistance, double targetAngle, String name) {
        this.drivetrain  = drivetrain;
        this.ll = ll;
        this.targetDistance = targetDistance;
        this.targetAngle = targetAngle;
        this.name = name;
        pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.8;
        MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.8;
        //addRequirements(drivetrain, ll);
    }

    public void execute() {
        double currentTargetTX = ll.getTargetTX();
        double currentTargetTY = ll.getTargetTY();
        double currentTargetAngle = ll.getTargetAngle();

        if (currentTargetTX != 0.0 || currentTargetTY != 0.0 || currentTargetAngle != 0.0) {
            lastValidTargetTX = currentTargetTX;
            lastValidTargetTY = currentTargetTY;
            lastValidTargetAngle = currentTargetAngle;
            lostDetectionTimer.reset();
        } else if (lostDetectionTimer.get() < lostDetectionTimeout) {
            currentTargetTX = lastValidTargetTX;
            currentTargetTY = lastValidTargetTY;
            currentTargetAngle = lastValidTargetAngle;
        } else {
            currentTargetTX = 0.0;
            currentTargetTY = 0.0;
            currentTargetAngle = 0.0;
        }

        double distanceError = targetDistance - currentTargetTY;
        double horizontalError = -currentTargetTX; // Invert TX for horizontal adjustment
        double distanceAdjust = limelight_range_proportional(distanceError);
        double horizontalAdjust = horizontalError * kP_horizontal;
        
        // Smoothing the horizontal adjustment to prevent jerky movement
        horizontalAdjust = (horizontalAdjust + lastHorizontalAdjust) / 2;
        lastHorizontalAdjust = horizontalAdjust;

        // Slow down the robot as it approaches the target
        // double speedFactor = Math.min(1.0, 2/Math.abs(distanceError / targetDistance));
        // distanceAdjust *= speedFactor;
        // horizontalAdjust *= speedFactor;

        double angleError = currentTargetAngle - targetAngle;
        double steeringAdjust = limelight_aim_proportional(angleError);

        System.out.println("AlignCommand executing");
        System.out.println("Distance Adjust: " + distanceAdjust);
        System.out.println("Horizontal Adjust: " + horizontalAdjust);
        System.out.println("Steering Adjust: " + steeringAdjust);

        drivetrain.setControl(
            pathApplyRobotSpeeds
            .withSpeeds(new ChassisSpeeds(distanceAdjust, horizontalAdjust, steeringAdjust)));

        System.out.println("Control Set:");
        System.out.println("VelocityX: " + distanceAdjust);
        System.out.println("VelocityY: " + horizontalAdjust);
        System.out.println("RotationalRate: " + steeringAdjust);
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() { 
        return (LimelightHelpers.getTXNC(name)) < 0.1 && (LimelightHelpers.getTYNC(name)) < 0.1;
    }

    double limelight_aim_proportional(double angleError) {
        double targetingAngularVelocity = angleError * kP_aim;
        System.out.println("Calculated targetingAngularVelocity: " + targetingAngularVelocity);
        targetingAngularVelocity *= MaxAngularRate;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    // Simple proportional ranging control with Limelight's "ty" value
    double limelight_range_proportional(double distanceError) {
        double targetingForwardSpeed = distanceError * kP_range;
        System.out.println("Calculated targetingForwardSpeed: " + targetingForwardSpeed);
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
}
