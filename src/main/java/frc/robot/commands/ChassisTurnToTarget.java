package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChassisTurnToTarget extends CommandBase 
{
  double m_headingError;
  double m_turn;

  public ChassisTurnToTarget()
  {
    /**
     * Require chassis to takeover drive train input.
     * This will end the driveWithJoystick command that will be recalled after this command ends.
     */
    addRequirements(RobotContainer.m_chassis);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    m_turn = 0.0;
  }
  
  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute()
  {
    /* Heading error is how off the robot is from the target in the "x" direction. */
    m_headingError = RobotContainer.m_limelight.getTargetOffsetX();

    /* Calculate turn based on offset from target. */
    if (m_headingError > Constants.CHASSIS_TURN_ERROR_THRESHOLD)
    {
      m_turn = Constants.K_CHASSIS_TURN_VISION_P * m_headingError - Constants.K_CHASSIS_TURN_VISION_MIN;
      RobotContainer.m_launcher.updateLaunchReadyStatus(3, false);
    }
    else if (m_headingError < -Constants.CHASSIS_TURN_ERROR_THRESHOLD)
    {
      m_turn = Constants.K_CHASSIS_TURN_VISION_P * m_headingError + Constants.K_CHASSIS_TURN_VISION_MIN;
      RobotContainer.m_launcher.updateLaunchReadyStatus(3, false);
    }
    else
    {
      m_turn = 0.0;
      RobotContainer.m_launcher.updateLaunchReadyStatus(3, true);
    }

    /* Give output to drive train. */
    RobotContainer.m_chassis.autoDrive(RobotContainer.m_OI.getDriverJoystick().getY(), m_turn);
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished()
  {
    /* Command will end when joystick button is released due to requirement of chassis class of stop command. */
    return false;
  }
}