package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LoadBallIntoChamber extends CommandBase 
{
  // The subsystem the command runs on
  private static boolean m_ballPresentEntering = false;
  private static boolean m_ballPresentExiting = false;

  private static int m_ballCount = 0;

  public LoadBallIntoChamber()
  {
    //addRequirements(m_subsystem);
  }

  @Override
  public void initialize()
  {
  }
  
  @Override
  public void execute()
  {
    // Check if ball is at end of chamber
    if (RobotContainer.m_chamber.SensorDistance(Constants.CHAMBER_EXITING_BALLPOS) < Constants.CHAMBER_BALL_NEAR_DIST)
    {
      RobotContainer.m_intake.setWheelRPM(0);
      m_ballPresentExiting = true;
    }
    else if (RobotContainer.m_chamber.SensorDistance(Constants.CHAMBER_EXITING_BALLPOS) > Constants.CHAMBER_BALL_FAR_DIST)
    {
      if (m_ballPresentExiting == true)
      {
        m_ballCount--;
        m_ballPresentExiting = false;
      }
    }
    else 
    {
      m_ballPresentExiting = false;
      if (RobotContainer.m_chamber.SensorDistance(Constants.CHAMBER_ENTERING_BALLPOS) < Constants.CHAMBER_BALL_FAR_DIST)
      {
        RobotContainer.m_chamber.stepChamberDistance(Constants.CHAMBER_BALL_STEP_DIST);
        if (m_ballPresentEntering == false)
        {
          m_ballCount++;
          m_ballPresentEntering = true;
        }
      else
      {
        m_ballPresentEntering = false;
      }
      }
    }
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }

  public int getBallCount()
  {
    return m_ballCount;
  }
}