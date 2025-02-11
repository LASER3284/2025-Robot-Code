// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
 
  private Pose2d start;

  private Pose2d end;
  private RobotContainer m_robotContainer;


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  public void disabledInit() {}

  public void disabledPeriodic() {
    
  }

  public void disabledExit() {}

  public void autonomousInit() {

   // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // start = m_robotContainer.drivetrain.getState().Pose;
     if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
     }
  }

  public void autonomousPeriodic() {

    // end = m_robotContainer.drivetrain.getState().Pose;
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
