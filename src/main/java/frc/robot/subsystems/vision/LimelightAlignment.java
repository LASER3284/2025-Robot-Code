// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.ObjectInputFilter.Config;
import java.security.PublicKey;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rollers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class LimelightAlignment extends SubsystemBase {
  private static LimelightAlignment instance;

  private final Drivetrain drivetrain  = Constants.SwerveConstants.createDrivetrain();
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
  

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric start = new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);

  /** Creates a new LimelightAlignment. */
  public LimelightAlignment() {
  }

  public static LimelightAlignment getInstance() {
        if (instance == null) {
            instance = new LimelightAlignment();
        }
        return instance;
    }

  public Command LimelightAlign(Drivetrain drivetrain, boolean left, String name){
    return runOnce(() -> this.driveAtTag(drivetrain, left, name));
  }
  
  public Command setYaw(double yaw){
    return run(() -> yawControl.setSetpoint(yaw));
  }

  // George Code
  private void driveAtTag(Drivetrain driveT, boolean left, String limelightname){
      drivetrain.applyRequest(() -> start);

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
      double yawSpeed = -yawControl.calculate(driveT.getPigeon2().getYaw().getValueAsDouble())  * 0.01;
      if(yawSpeed < 0.05){
        yawSpeed = 0;
      }
      
      //driveT.applyRequest(() ->
      //  robotCentricRequest.withVelocityX(xSpeed).withVelocityY(ySpeed)
      //);

      // if tx or ty is not 0, then move
      
      driveT.setControl(new SwerveRequest.RobotCentric().withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(yawSpeed));
      
      
      //System.out.println("X Speed: " + xSpeed + " Y Speed: " + ySpeed + " X Pos: " + cameraPose_TargetSpace.getX() + " Y Pos: " + cameraPose_TargetSpace.getY());

  }

  @Override
  public void periodic() {
    // Basic targeting data
    double tx = LimelightHelpers.getTX(limelightname);  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY(limelightname);  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA(limelightname);  // Target area (0% to 100% of image)
    boolean tv = LimelightHelpers.getTV(limelightname); // Do you have a valid target?

    // System.out.println("tx: " + tx + " ty: " + ty + " ta: " + ta + " tv: " + tv);
  }
}