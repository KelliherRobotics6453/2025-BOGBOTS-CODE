// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Timer autoTimer = new Timer();
  private double autoStartTime;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    autoStartTime = autoTimer.get();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    
    if (autoTimer.get() - autoStartTime < AutoConstants.AutoRunTime) {
      m_robotContainer.drive.withVelocityX(-(DriveConstants.adjustSpeed(AutoConstants.ySpeed))) // Drive forward with negative Y (forward)// speed was getting multipled by MaxSpeed.
        .withVelocityY(-(DriveConstants.adjustSpeed(AutoConstants.xSpeed))) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      ;
    } else {
      m_robotContainer.drive.withVelocityX(0) // Drive forward with negative Y (forward)// speed was getting multipled by MaxSpeed.
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      ;
    }
   
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
