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
    /* Set Limelight to vision mode to track target. */
    RobotContainer.m_limelight.setModeVision();

    m_turn = 0.0;
  }
  
  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute()
  {
    /* Heading error is how off the robot is from the target in the "x" direction. */
    m_headingError = -RobotContainer.m_limelight.getTargetOffsetX();

    /* Calculate turn based on offset from target. */
    if (m_headingError > 0.5)
    {
      m_turn = Constants.K_CHASSIS_TURN_VISION_P * m_headingError - Constants.K_CHASSIS_TURN_VISION_MIN;
    }
    else if (m_headingError < -0.5)
    {
      m_turn = Constants.K_CHASSIS_TURN_VISION_P * m_headingError + Constants.K_CHASSIS_TURN_VISION_MIN;
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
    return (m_headingError < 0.5 && m_headingError > -0.5);
  }
}