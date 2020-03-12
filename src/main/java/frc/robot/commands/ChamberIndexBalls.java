package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ChamberIndexBalls extends CommandBase 
{
  double m_enteringRange;
  double m_exitingRange;

  public ChamberIndexBalls()
  {
    addRequirements(RobotContainer.m_chamber);

    m_enteringRange = 0.0;
    m_exitingRange = 0.0;
  }

  @Override
  public void initialize()
  {
  }
  
  @Override
  public void execute()
  {
    m_enteringRange = RobotContainer.m_chamber.getEnteringRange();
    m_exitingRange = RobotContainer.m_chamber.getExitingRange();

    if (m_exitingRange >= Constants.CHAMBER_BALL_THRESHOLD) // space is still available in chamber --> run chamber if needed.
    {
      if (m_enteringRange < Constants.CHAMBER_BALL_THRESHOLD) // ball is present at bottom of the chamber --> run chamber and index.
      {
        RobotContainer.m_chamber.stepChamberDistance(Constants.CHAMBER_BALL_STEP_DIST);
      }
      else if (m_enteringRange >= Constants.CHAMBER_BALL_THRESHOLD) // no ball is present at bottom of the chamber --> stop chamber.
      {
        RobotContainer.m_chamber.setElevatorPower(0.0);
      }
      else
      {
        RobotContainer.m_chamber.setElevatorPower(0.0);
        System.out.println("\nUnable to read entering chamber ultrasonic. Chamber was automatically stopped.\n");
      }
    }
    else if (m_exitingRange < Constants.CHAMBER_BALL_THRESHOLD) // ball is at top of the chamber ready to launch --> don't run chamber.
    {
      RobotContainer.m_chamber.setElevatorPower(0.0);
    }
    else
    {
      RobotContainer.m_chamber.setElevatorPower(0.0);
      System.out.println("\nUnable to read exiting chamber ultrasonic. Chamber was automatically stopped.\n");
    }
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}