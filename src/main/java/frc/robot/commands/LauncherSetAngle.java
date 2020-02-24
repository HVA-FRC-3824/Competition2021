package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LauncherSetAngle extends CommandBase 
{
  double m_setpoint;
  double m_angleError;
  double m_pivotOutput;

  public LauncherSetAngle(int setpoint)
  {
    addRequirements(RobotContainer.m_launcher);

    m_setpoint = setpoint;
  }

  @Override
  public void initialize()
  {
    m_angleError = 0.0;
    m_pivotOutput = 0.0;
  }
  
  @Override
  public void execute()
  {
    /* Angle error is how off the robot is from the setpoint. */
    m_angleError = m_setpoint - RobotContainer.m_launcher.getPivotADC();

    /* Calculate output to launcher angle pivot based on offset from target. */
    if (m_angleError > Constants.LAUNCHER_PIVOT_ADC_THRESHOLD)
    {
      m_pivotOutput = Constants.LAUNCHER_PIVOT_ANGLE_P * m_angleError - Constants.LAUNCHER_AIM_VISION_MIN;
    }
    else if (m_angleError < -Constants.LAUNCHER_PIVOT_ADC_THRESHOLD)
    {
      m_pivotOutput = Constants.LAUNCHER_PIVOT_ANGLE_P * m_angleError + Constants.LAUNCHER_AIM_VISION_MIN;
    }

    /* Give output to launcher pivot. */
    RobotContainer.m_launcher.setPivotPower(m_pivotOutput);
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return (m_angleError < Constants.LAUNCHER_PIVOT_ADC_THRESHOLD && m_angleError > -Constants.LAUNCHER_PIVOT_ADC_THRESHOLD);
  }
}