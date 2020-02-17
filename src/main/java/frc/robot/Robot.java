/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  /**
   * Declare autonomous command object to later be instantiated and scheduled based on chooser in RobotContainer.java.
   */
  private Command m_autonomousCommand;

  /**
   * Declare object for RobotContainer.java.
   * This object will be used when desiring to perform an action requiring an external subsystem, command, etc.
   * Because of the static attribute, other classes can use this instance with just RobotContainer.desiredAction.
   */
  public static RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    /**
     * By instantiating the RobotContainer, all other initializations will be performed.
     * The RobotContainer object will also be ready for use throughout the project.
     */
    m_robotContainer = new RobotContainer();

    CameraServer.getInstance().startAutomaticCapture(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    /**
     * Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
     * commands, running already-scheduled commands, removing finished or interrupted commands,
     * and running subsystem periodic() methods.  This must be called from the robot's periodic
     * block in order for anything in the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    /* Turn off Limelight LED when disabled so it doesn't blind drive team. */
    RobotContainer.m_limelight.turnOffLED();
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic()
  {
  }

  /**
   * This function is called at the start of autonomous.
   */
  @Override
  public void autonomousInit()
  {
    /* Zero robot heading to current heading. */
    RobotContainer.m_chassis.zeroHeading();
    RobotContainer.m_chassis.resetEncoders();

    /**
     * Gets selected autonomous command from autoChooser and schedules said command.
     */
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null)
      m_autonomousCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  /**
   * This function is called at the start of operator control.
   */
  @Override
  public void teleopInit() 
  {
    /**
     * This makes sure that the autonomous stops running when
     * teleop starts running. If you want the autonomous to
     * continue until interrupted by another command, remove
     * this line or comment it out.
     */
    if (m_autonomousCommand != null)
      m_autonomousCommand.cancel();

    /* Initially switch Limelight to driver mode for teleoperation. */
    // RobotContainer.m_limelight.setModeDriver();
    RobotContainer.m_limelight.setModeVision();

    /**
     * Initialize default commands for all subsystems.
     * Do this in teleopInit rather than robotInit or autonomousInit because default commands 
     * will interfere with autonomous commands.
     */
    RobotContainer.initializeDefaultCommands();

    RobotContainer.m_chamber.setChamberBasePower(0.2);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
  }

  /**
   * This function is called when test mode is enabled.
   */
  @Override
  public void testInit()
  {
    /**
     * Cancels all running commands at the start of test mode.
     */
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }
}