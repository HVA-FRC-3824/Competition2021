package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LoadBallIntoChamber extends CommandBase 
{
  // The subsystem the command runs on

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
    if (RobotContainer.m_chamber.SensorDistance(Constants.CHAMBER_EXITING_BALLPOS) < Constants.CHAMBER_BALL_PRESENT_DIST)
    {
        RobotContainer.m_chamber.setChamberElevatorRMP(0);
    }

    // Check entering sensor for ball
    else 
    {
      if (RobotContainer.m_chamber.SensorDistance(Constants.CHAMBER_ENTERING_BALLPOS) < Constants.CHAMBER_BALL_PRESENT_DIST)
      {
          RobotContainer.m_chamber.stepChamberDistance(Constants.CHAMBER_BALL_STEP_DIST);
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
}