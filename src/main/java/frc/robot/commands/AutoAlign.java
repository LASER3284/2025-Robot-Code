package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AutoAlign extends Command{
    private Drivetrain drivetrain;
    private LimelightHelpers ll;

    private final SwerveRequest.RobotCentric m_alignRequest;

    private final double MaxSpeed;
    private final double MaxAngularRate;

    private final double targetDistance; // Desired distance from the tag
    private final double targetAngle; // Desired angle relative to the tag

    private static final double kP_aim = 0.005; // Proportional gain for aiming
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


    public void initialize() {}

    public AutoAlign(Drivetrain drivetrain, LimelightHelpers ll, double targetDistance, double targetAngle) {
        this.drivetrain  = drivetrain;
        this.ll = ll;
        this.targetDistance = targetDistance;
        this.targetAngle = targetAngle;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
        MaxSpeed = Elevator.MaxSpeed * 0.5;
        MaxAngularRate = Elevator.MaxAngularRate * 0.5;
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
            m_alignRequest
                .withVelocityX(distanceAdjust)  // Forward/backward movement
                .withVelocityY(horizontalAdjust) // Horizontal (lateral) movement
                .withRotationalRate(steeringAdjust) // Rotational correction
        );

        System.out.println("Control Set:");
        System.out.println("VelocityX: " + distanceAdjust);
        System.out.println("VelocityY: " + horizontalAdjust);
        System.out.println("RotationalRate: " + steeringAdjust);
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() { return true; }

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
