package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;

public class AutoAlign3 extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private Drivetrain drivetrain;
    private double tagID = -1;
    private String limelightName;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AutoAlign3(boolean isRightScore, Drivetrain drivetrain, String limelightName) {
        xController = new PIDController(0.5, 0, 0);
        yController = new PIDController(0.5, 0, 0);
        rotController = new PIDController(0.5, 0, 0);
        this.isRightScore = isRightScore;
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;

        addRequirements(drivetrain);
    }

    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        rotController.setSetpoint(0);
        rotController.setTolerance(0.5);

        xController.setSetpoint(-0.5);
        xController.setTolerance(0.005);

        yController.setSetpoint(0.19);
        yController.setTolerance(0.005);

        if (LimelightHelpers.getTV(limelightName)) {
            tagID = LimelightHelpers.getFiducialID(limelightName);
        }
    }

    public void execute() {
        if (LimelightHelpers.getTV(limelightName)) {
            this.dontSeeTagTimer.reset();
        }

        double[] positions = LimelightHelpers.getBotPose_TargetSpace(limelightName);
        SmartDashboard.putNumber("x", positions[2]);

        double xSpeed = xController.calculate(positions[2]);
        SmartDashboard.putNumber("xspeed", xSpeed);
        double ySpeed = -yController.calculate(positions[0]);
        double rotValue = -rotController.calculate(positions[4]);

        drivetrain.applyRequest(() ->
            drive.withVelocityX(xSpeed)
                 .withVelocityY(ySpeed)
                 .withRotationalRate(rotValue)
        );

        if (!rotController.atSetpoint() ||
            !yController.atSetpoint() ||
            !xController.atSetpoint()) {
                stopTimer.reset();
            }
        else {
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(0));
        }
    }

    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)  
        );
    }

    public boolean isFinished() {
        return this.dontSeeTagTimer.hasElapsed(4) ||
            stopTimer.hasElapsed(4);
    }
}
