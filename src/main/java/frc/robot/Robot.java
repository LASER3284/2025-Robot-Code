// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.elevator.ToHome;
import frc.robot.commands.elevator.ToPosition;
import frc.robot.commands.pivot.PivotToAngle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JS;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.pivotintake.IntakeRollers;
import frc.robot.subsystems.pivotintake.Pivot;
import frc.robot.subsystems.vision.LimelightHelpers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;


public class Robot extends TimedRobot {
  private JS js;
  private Rollers rollers;
  private IntakeRollers irollers;
  private Pivot pintake;
  private Elevator elevator;

  private Command m_autonomousCommand;
 
  private Pose2d start;

  private Pose2d end;
  private RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;


  public Robot() {
    m_robotContainer = new RobotContainer();

    NamedCommands.registerCommand("source intake", new SourceIntake(js, rollers));
    NamedCommands.registerCommand("ground intake", new CoralIntake(js, irollers, pintake, -0.25, -7.5)
            .andThen(irollers.setMotorSpeed_command(0.4)
            .andThen(rollers.coral_roller_on_command(-0.8)))
            .andThen(new PivotToAngle(js, Degrees.of(0.2))));
    NamedCommands.registerCommand("L2 Score", new ToPosition(elevator, Inches.of(2), Inches.of(3)));
    NamedCommands.registerCommand("L3 Score", new ToPosition(elevator, Inches.of(3), Inches.of(4)));
    NamedCommands.registerCommand("L4 Score", new ToPosition(elevator, Inches.of(4), Inches.of(5)));
    NamedCommands.registerCommand("Home", new ToHome(elevator, Inches.of(0), Inches.of(0)));

  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  public void disabledInit() {}

  public void disabledPeriodic() {
    
  }

  public void disabledExit() {}

  public void autonomousInit() {

   m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    start = m_robotContainer.drivetrain.getState().Pose;
     if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
     }
  }

  public void autonomousPeriodic() {

    end = m_robotContainer.drivetrain.getState().Pose;
    SmartDashboard.putNumber("original x pose", start.getX());
    SmartDashboard.putNumber("original y pose", start.getY());
    double x = Math.abs(start.getX() - end.getX());
    double y = Math.abs(start.getY() - end.getY());
    SmartDashboard.putNumber("end x pose", end.getX());
    SmartDashboard.putNumber("end y pose", end.getY());
    SmartDashboard.putString("Distances",String.format("Delta X : (%.4f)m Delta Y : (%.4f)m ",x,y) );

    CommandScheduler.getInstance().run(); 

  }

  public void autonomousExit() {}

  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void teleopPeriodic() {}

  public void teleopExit() {}

  public void testInit() {
    
  }

  public void testPeriodic() {}

  public void testExit() {}

  public void simulationPeriodic() {}

 
}
