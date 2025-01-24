// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;


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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  public void autonomousPeriodic() {

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
    CommandScheduler.getInstance().cancelAll();
  }

  public void testPeriodic() {}

  public void testExit() {}

  public void simulationPeriodic() {}
}
