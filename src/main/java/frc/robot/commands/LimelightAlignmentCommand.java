package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.LimelightAlignment;
import frc.robot.subsystems.vision.LimelightHelpers;

public class LimelightAlignmentCommand extends Command {
    private static LimelightAlignment instance;

    //private final Drivetrain drivetrain  = Constants.SwerveConstants.createDrivetrain();
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private Boolean run = false;

    //PID bad for Limelight don't use
    private final PIDController xControl = new PIDController(1, 0, 0.3);
    private final PIDController zControl = new PIDController(1, 0, 0.3);
    private final PIDController yawControl = new PIDController(0.3, 0, 0.2);
    private final double kix = 1.5;
    private final double kiy = 1.5;
    private String limelightname;

    double ySpeed = 0;
    double xSpeed = 0;

    double yaw;
    boolean left;
    String limelightName;

    
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric start = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

    
    public LimelightAlignmentCommand(double yaw, boolean left, String limelightName) {
        this.yaw = yaw;
        this.left = left;
        this.limelightName = limelightName;
    }

    public void initialize() {
        yawControl.setSetpoint(yaw);
    }

    public void execute() {
      RobotContainer.drivetrain.applyRequest(() -> start);

     Pose3d cameraPose_TargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(limelightname); // Camera's pose relative to tag (should use Robot's pose in the future)
   
      // when basing speed off offsets lets add an extra proportional term for each of these
      // lets not edit the yaw
      double xPos = -0.16501350439035187;
      double yPos = -0.08830078668779259;

      xControl.setSetpoint(0);
      zControl.setSetpoint(1);
      xControl.setTolerance(0.05);
      zControl.setTolerance(0.05);
  
      yawControl.setTolerance(0.05);
      yawControl.enableContinuousInput(-180, 180);

      // when lime light is not seeing the target, it will return 0 for x and y
      // if x and y are 0, then we should not move
      double xOffset = LimelightConstants.rightOffset;
      if(left){
        xOffset = LimelightConstants.leftOffset;
      }
      if (cameraPose_TargetSpace.getX() != 0 && cameraPose_TargetSpace.getY() != 0)
      {
        ySpeed = kiy * (cameraPose_TargetSpace.getX() + xOffset);
        xSpeed = -kix * (cameraPose_TargetSpace.getZ() + LimelightConstants.yOffset);
      }
      else
      {
        System.out.println("can't see apriltag");
        ySpeed = 0;
        xSpeed = 0;
      }

      //System.out.println("rot: " + cameraPose_TargetSpace.getRotation());
      double yawSpeed = -yawControl.calculate(RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble())  * 0.01;
      if(yawSpeed < 0.05){
        yawSpeed = 0;
      }
      
      RobotContainer.drivetrain.applyRequest(() ->
       robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed)
      );

      // if tx or ty is not 0, then move
      
      RobotContainer.drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(yawSpeed));
      
      
      System.out.println("X Speed: " + xSpeed + " Y Speed: " + ySpeed + " X Pos: " + cameraPose_TargetSpace.getX() + " Y Pos: " + cameraPose_TargetSpace.getY());

    }

    public void end() {
        //driveT.setControl(start);
    }

    public boolean isFinished() {
        return LimelightHelpers.getTX(limelightName) < 0.5 && LimelightHelpers.getTY(limelightName) < 0.5;
    }
}
